from queue import PriorityQueue
import queue
import subprocess
import signal
import time
import threading
import os
import sys
import asyncio
import websockets
import json
class Mission:
    def __init__(self, status,keyword, filename,req):
        self.filename = filename
        self.status = status
        self.priority = sys.maxsize
        self.keyword = keyword
        self.popen = None
        self.Mid = -1
        self._terminate_flag = False
        self.requirements=json.dumps(req)

    def run(self):
        try:
            p = subprocess.Popen(["python", self.filename],)
            stdout, stderr = p.communicate(input=self.requirements.encode('utf-8'))
            self.popen = p
            self.Mid = p.pid
            t = threading.Thread(target=self.statusUpdater)
            t.start()
        except FileNotFoundError as e:
            print(f"File {self.filename} not found: {e}")
        except PermissionError as e:
            print(f"Permission denied for {self.filename}: {e}")
        except Exception as e:
            print(f"Initiation failed: {e}")

    def statusUpdater(self):
        while not self._terminate_flag:
            if self.popen.poll() is None:
                if self.status != "paused":
                    self.status = "running"
            elif self.status == "paused":
                time.sleep(0.5)
            else:
                self.status = "finished"
                break
            time.sleep(0.5)

    def pause(self):
        if self.status == "running":
            try:
                os.kill(self.Mid, signal.SIGSTOP)
                self.status = "paused"
            except Exception as e:
                print("Pausing failed: ", e)

    def resume(self):
        if self.status == "paused":
            try:
                os.kill(self.Mid, signal.SIGCONT)
                self.status = "running"
            except Exception as e:
                print("Resume failed: ", e)

    def terminate(self):
        try:
            self._terminate_flag = True
            self.popen.terminate()
            self.popen.wait()
            self.status = "killed"
        except Exception as e:
            print("Termination failed: ", e)



class MP(threading.Thread):
    def __init__(self):
        super().__init__()
        self.q = PriorityQueue()
        self.p=[]
        self.curr = Mission("stabilizing", "stabilizer", "stabilizer.py",None)
        self.curr.run()
        self.begin = False

    def run(self):
        while self.begin:

            if (self.q.empty() and not self.curr.keyword=="landing") or (self.curr.status == "finished" and self.curr.keyword != "stabilizer"):
                self.p.remove(self.curr.priority)
                self.curr = Mission("stabilizing","stabilizer", "stabilizer.py",None)
                self.curr.run()

            if not self.q.empty():
                priority, prior_mission = self.q.get()
                if priority < self.curr.priority:
                    self.curr.pause()
                    if self.curr.status == "paused":
                        self.q.put((self.curr.priority, self.curr))


                    self.curr = prior_mission
                    if self.curr.status == "paused":
                        self.curr.resume()
                    else:
                        self.curr.run()
                        self.p.append(self.curr.priority)

                else:
                    self.q.put((prior_mission.priority, prior_mission))
            time.sleep(0.5)

    def priority_assigner(self,mission):
        curr_q = list(self.q.queue).sort(key=lambda x:x[0])
        if curr_q:
            if mission.keyword=="Emergency":
                mission.priority = curr_q[0][0]-1
            elif mission.keyword=="series":
                mission.priority = self.curr.priority+1
        else:
            mission.priority=0


MissionPlanner = MP()
MissionPlanner.begin = True
MissionPlanner.start()


async def GroundStation_to_Drone():
    url = "ws://<ground-station-ip>:8765"
    async with websockets.connect(url) as websocket:
        while True:
            data= await websocket.recv()
            instruction= json.loads(data)
            if instruction["type"]=="emergency_mission":
                if instruction["mission"]=="rolling back":
                    while not MissionPlanner.q.empty():
                        MissionPlanner.q.get()
                    m = Mission("running","Emergency","rollback.py",instruction["requirements"])
                    MissionPlanner.priority_assigner(m)
                    MissionPlanner.q.put((m.priority,m))
                elif instruction["mission"]=="landing":
                    while not MissionPlanner.q.empty():
                        MissionPlanner.q.get()
                    m=Mission("running","Emergency","safe_landing.py",instruction["requirements"])
                    MissionPlanner.priority_assigner(m)
                    MissionPlanner.q.put((m.priority,m))
                elif instruction["mission"]=="command":
                    while not MissionPlanner.q.empty():
                        MissionPlanner.q.get()
                    m=Mission("running","Emergency","dem o.py",instruction["requirements"])
                    MissionPlanner.priority_assigner(m)
                    MissionPlanner.q.put((m.priority,m))

            elif instruction["type"]=="mission":
                if instruction["mission"]=="shooting":
                    m=Mission("running",instruction["keyword"],"shooting.py",instruction["requirements"])
                    MissionPlanner.priority_assigner(m)
                    MissionPlanner.q.put((m.priority,m))
                elif instruction["mission"]=="follow me":
                    m=Mission("running",instruction["keyword"],"follow_me.py",instruction["requirements"])
                    MissionPlanner.priority_assigner(m)
                    MissionPlanner.q.put((m.priority,m))

                elif instruction["mission"]=="exploring":
                    m = Mission("running",instruction["keyword"],"exploring.py",instruction["requirements"])
                    MissionPlanner.priority_assigner(m)
                    MissionPlanner.q.put((m.priority,m))

                elif instruction["mission"]=="demo":
                    m = Mission("running",instruction["keyword"],"demo.py",instruction["requirements"])
                    MissionPlanner.priority_assigner(m)
                    MissionPlanner.q.put((m.priority,m))

                elif instruction["mission"]=="Basic func":
                    m =  Mission("running",instruction["keyword"],"BasicFunc.py",instruction["requirements"])
                    MissionPlanner.priority_assigner(m)
                    MissionPlanner.q.put((m.priority,m))




asyncio.run(GroundStation_to_Drone())

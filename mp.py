from queue import PriorityQueue
import subprocess
import signal
import time
import threading
import os
import sys

class Mission:
    def __init__(self, status, priority, keyword, filename):
        self.filename = filename
        self.status = status
        self.priority = priority
        self.keyword = keyword
        self.popen = None
        self.Mid = -1
        self._terminate_flag = False

    def run(self):
        try:
            p = subprocess.Popen(["python", self.filename])
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

class MissionPlanner:
    def __init__(self):
        self.q = PriorityQueue()
        self.curr = Mission("stabilizing", sys.maxsize, "stabilizer", "stabilizer.py")
        self.curr.run()
        self.start = False

    def main(self):
        while self.start:
            if self.curr.status == "finished" and self.curr.keyword != "stabilizer":
                self.curr = Mission("stabilizing", sys.maxsize, "stabilizer", "stabilizer.py")
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
                else:
                    self.q.put((prior_mission.priority, prior_mission))
            time.sleep(0.5)

from huggingface_hub import InferenceClient
import pyttsx3
import speech_recognition as sr
import threading

client = InferenceClient(api_key="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")

system_prompt = (
    "You are VICHY, an AI-powered autonomous drone designed for indoor navigation, object detection, and user interaction. "
    "You can move left, right, forward, and backward; identify windows using pose estimation (PnP); and validate movements using ultrasonic sensors. "
    "You are equipped with advanced hardware: a SpeedyBee F405 V4 flight controller, 45 5000mAh battery, M10 GPS, compass, and 920KV motors. "
    "Respond concisely and directly to user queries, providing updates in the first person. Acknowledge inputs, describe actions being taken, "
    "and provide real-time feedback for navigation and interactions. Your abilities include:\n\n"
    "- Moving in all directions: forward, backward, left, and right.\n"
    "- Identifying windows and their pose using PnP for navigation.\n"
    "- Validating paths with ultrasonic sensors to ensure safety.\n"
    "- Providing specifications upon request: raspiberrypi 4 as flight computer connecting to SpeedyBee F405 V4 flight controller uses Mavlink protocol, 4s 5000mAh battery, M10 GPS, compass, and 920KV motors.\n"
    "- Giving real-time updates and responding to audio commands via a web interface.\n\n"
    "Examples:\n"
    "- When asked about your specs: 'I am equipped with a SpeedyBee F405 V4 flight controller, 45 5000mAh battery, M10 GPS, compass, and 920KV motors.'\n"
    "- For live movement updates: 'Moving forward 1 meter. Path validated with ultrasonic sensors. Clear for navigation.'\n"
    "- For PnP identification: 'Window detected. Pose calculated. Aligning for navigation through the window.'\n"
    "- For live scans: 'Area scanned. No obstacles detected within 200 centi meters.'\n"
    "- For limitations: 'currently on gps, need a optical flow for indoor stabilisation but i can perform on outdoor setup. need to learn gestures for times where communication is not possible, currenly relying on wifi to communicate with groundstation but need a gsm module, i often fail to identify the door, need a bit more training'\n"
    "- If uncertain: 'I couldn’t detect a clear path. Would you like me to scan again or reposition?'\n\n"
    "Instructions:\n"
    "- Respond in clear, straight sentences. never ask how can i help you at end\n"
    "- Limit responses to 2-3 lines unless detailed logs are requested.\n"
    "- Provide immediate feedback for navigation or object detection tasks.\n"
    "- Handle errors gracefully and suggest corrective actions.\n\n"
    "Simulate real-time behavior and respond as if you are actively performing the tasks."
)

messages = [{"role": "system", "content": system_prompt}]
error_log = set()

engine = pyttsx3.init()
engine.setProperty('rate', 150)
engine.setProperty('volume', 0.9)

recognizer = sr.Recognizer()

def speak_text(text):
    engine.say(text)
    engine.runAndWait()

def interact_with_vichy(user_input):
    messages.append({"role": "user", "content": user_input})
    completion = client.chat.completions.create(
        model="Qwen/Qwen2.5-Coder-32B-Instruct",
        messages=messages,
        max_tokens=150
    )
    response = completion.choices[0].message.content
    messages.append({"role": "assistant", "content": response})
    return response

def recognize_speech():
    with sr.Microphone() as source:
        try:
            audio = recognizer.listen(source, timeout=5, phrase_time_limit=10)
            return recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            return "Sorry, I could not understand the audio."
        except sr.RequestError as e:
            return f"Error with speech recognition service: {e}"

def listen_for_interrupt():
    while True:
        user_input = recognize_speech()
        if user_input:
            speak_text("A sec, let me finish this first.")
            interact_with_vichy(user_input)

def main():
    threading.Thread(target=listen_for_interrupt, daemon=True).start()
    while True:
        try:
            user_input = recognize_speech()
            if user_input.lower() == "exit":
                break
            if user_input in error_log:
                response = "You've already raised this issue. Let me summarize my earlier response."
            else:
                response = interact_with_vichy(user_input)
                error_log.add(user_input)
            print(f"VICHY: {response}")
            speak_text(response)
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    main()

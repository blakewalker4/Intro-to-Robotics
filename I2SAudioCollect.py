import os
import serial
import wave

PORT = '/dev/ttyACM0'  # change to your port
BAUD = 115200
SAMPLE_RATE = 16000
RECORD_SECONDS = 10
NUM_SAMPLES = SAMPLE_RATE * RECORD_SECONDS

def get_next_filename(directory, label):
    existing = [f for f in os.listdir(directory) if f.startswith(label) and f.endswith('.wav')]
    index = len(existing) + 1
    return os.path.join(directory, f"{label}.{index}.wav")

def record_audio(ser, filename):
    print(f"Recording '{filename}' for {RECORD_SECONDS} seconds...")
    
    # Flush anything sitting in the buffer before we start
    ser.reset_input_buffer()

    raw = ser.read(NUM_SAMPLES * 2)

    with wave.open(filename, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(raw)

    print(f"Saved: {filename}")

def collect_samples(ser):
    while True:
        sample_name = input("Enter sample name (e.g., PersonA): ").strip()
        if not sample_name:
            print("Sample name cannot be empty.")
            continue

        sample_dir = os.path.join(os.getcwd(), sample_name)
        os.makedirs(sample_dir, exist_ok=True)
        print(f"Directory: {sample_dir}")

        while True:
            label = input("Enter label to record (e.g., yes, no): ").strip()
            if not label:
                print("Label cannot be empty.")
                continue

            while True:
                filename = get_next_filename(sample_dir, label)
                input(f"Press Enter to record '{label}'...")
                record_audio(ser, filename)

                cont = input("Record another? (yes/no): ").strip().lower()
                if cont != 'yes':
                    break

            next_label = input("Record a different label? (yes/no): ").strip().lower()
            if next_label != 'yes':
                break

        next_sample = input("New sample directory? (yes/no): ").strip().lower()
        if next_sample != 'yes':
            print("Done.")
            break

if __name__ == "__main__":
    ser = serial.Serial(PORT, BAUD, timeout=15)
    print("Connected. ESP is streaming continuously.")
    
    try:
        collect_samples(ser)
    finally:
        ser.close()
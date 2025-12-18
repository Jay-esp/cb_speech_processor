import os
import struct
import serial
import time
from tkinter import Tk, filedialog
from pydub import AudioSegment

def extract_left_channel(input_file, output_file, sample_rate=48000):
    print("Extracting left channel from MP3...")
    # Load the MP3 file
    audio = AudioSegment.from_mp3(input_file)

    # Set the sample rate to 48 kHz
    audio = audio.set_frame_rate(sample_rate)

    # Extract the left channel (mono)
    left_channel = audio.split_to_mono()[0]

    # Get the raw samples (16-bit)
    samples = left_channel.get_array_of_samples()

    # Normalize and convert to 24-bit (aligned to 32-bit)
    max_16bit = 32768  # Maximum value for 16-bit audio
    max_24bit = 8388608  # Maximum value for 24-bit audio

    # Export the left channel as 32-bit raw PCM data
    with open(output_file, 'wb') as f:
        print(f"Writing 32-bit PCM data to {output_file}...")
        for sample in samples:
            # Normalize the 16-bit sample to 24-bit range
            sample_24bit = int((sample / max_16bit) * max_24bit)

            # Ensure the sample is within 24-bit range
            sample_24bit = max(-max_24bit, min(sample_24bit, max_24bit - 1))

            # Pack as 32-bit little-endian (24-bit aligned to upper bits, LSB = 0)
            sample_32bit = struct.pack('<i', sample_24bit << 8)
            f.write(sample_32bit)

        # Calculate the required padding
        file_size = f.tell()
        padding_size = (24576 - (file_size % 24576)) % 24576

        # Pad the file with zeros
        if padding_size > 0:
            print(f"Padding file with {padding_size} bytes...")
            f.write(b'\x00' * padding_size)

    print("File conversion complete!")

def send_file_over_serial(port, baudrate, file_path):
    print(f"Opening serial connection to {port} at {baudrate} baud...")
    # Open serial connection
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)  # Wait for ESP32 to initialize

    # Send file name (prepend '/' to indicate root directory)
    file_name = "/" + os.path.basename(file_path)  # Add leading slash
    print(f"Sending file name: {file_name}")
    ser.write(file_name.encode() + b'\n')  # Send file name

    # Wait for ESP32 to be ready
    print("Waiting for ESP32 to be ready...")
    while True:
        response = ser.readline().decode().strip()
        print(f"Received response: {response}")
        if response == "Ready to receive file. Send data now...":
            print("ESP32 is ready. Starting file transfer...")
            break

    # Send file data
    file_size = os.path.getsize(file_path)
    bytes_sent = 0
    with open(file_path, 'rb') as f:
        while True:
            chunk = f.read(1024)  # Read 1024 bytes at a time
            if not chunk:
                break
            ser.write(chunk)
            bytes_sent += len(chunk)
            print(f"Sent {bytes_sent}/{file_size} bytes ({bytes_sent/file_size*100:.2f}%)")
            time.sleep(0.1)  # Small delay to prevent overwhelming the ESP32

    # Send end-of-transfer marker (0xFFFFFFFF)
    end_marker = b'\xFF\xFF\xFF\xFF'
    ser.write(end_marker)
    print("End-of-transfer marker sent.")

    # Close serial connection
    ser.close()
    print("File transfer complete!")

def select_file(title, filetypes):
    # Create a Tkinter root window (hidden)
    root = Tk()
    root.withdraw()  # Hide the root window

    # Open a file dialog to select a file
    file_path = filedialog.askopenfilename(title=title, filetypes=filetypes)

    # Check if a file was selected
    if not file_path:
        print("No file selected. Exiting.")
        exit(1)

    return file_path

if __name__ == "__main__":
    # Step 1: Select MP3 file
    mp3_file = select_file("Select an MP3 file", [("MP3 Files", "*.mp3")])

    # Step 2: Convert MP3 to .bin
    bin_file = os.path.splitext(mp3_file)[0] + ".bin"
    extract_left_channel(mp3_file, bin_file)

    # Step 3: Send .bin file over serial
    port = "COM3"  # Replace with your ESP32's serial port
    baudrate = 115200
    send_file_over_serial(port, baudrate, bin_file)
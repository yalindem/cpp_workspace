import subprocess
import os

SOURCE = "main.cpp"
OUTPUT = "main"

# Derleme komutu
compile_cmd = ["g++", "-std=c++20", "-Wall", "-Wextra", "-O2", SOURCE, "-o", OUTPUT]

print("Building...")
result = subprocess.run(compile_cmd, capture_output=True, text=True)

if result.returncode != 0:
    print("❌ Error:\n", result.stderr)
else:
    print("✅ Success! Running...\n")
    subprocess.run([f"./{OUTPUT}"])
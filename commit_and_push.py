import subprocess
import argparse
import sys

SOURCE = "main.cpp"

# Argümanları oku
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--commit", help="Commit message", required=True)
parser.add_argument("-a", "--amend", help="Commit message", required=False)
args = parser.parse_args()
commit_message = args.commit
is_amend = args.amend

# Git komutları
if not is_amend:
    cmds = [
        ["git", "add", SOURCE],
        ["git", "commit", "-m", commit_message],
        ["git", "push"]
    ]
else:
    cmds = [
        ["git", "add", SOURCE],
        ["git", "commit", "-m", commit_message, "--amend"],
        ["git", "push", "-f"]
    ]    

for cmd in cmds:
    print(f"🔹 Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print("❌ Hata oluştu:\n", result.stderr)
        sys.exit(1)  # Hata durumunda döngüyü durdur
    else:
        if result.stdout.strip():
            print("✅ Başarılı:\n", result.stdout)
        else:
            print("✅ Başarılı!")
import server
import telemetry
import subprocess

def processfunc():
    subprocess.Popen(["python3", "server.py"])
    subprocess.Popen(["python3", "telemetry.py"])

if __name__ == '__main__':
    processfunc()

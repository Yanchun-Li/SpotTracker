import subprocess

def start_gstreamer_stream():
    command = "raspivid -o - -t 0 -hf -w 640 -h 480 -fps 24 | cvlc -vvv stream:///dev/stdin --sout '#rtp{sdp=rtsp://:8554/}' :demux=h264"
    subprocess.Popen(command, shell=True)

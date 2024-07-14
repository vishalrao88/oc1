from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput


#net = detectNet(model="/home/jetson/dev/oc1/models/ssd-mobilenet.onnx", labels="/home/jetson/dev/oc1/models/labels.txt", input_blob="input_0", output_cvg="scores", output_bbox="boxes", threshold=0.5)


#net = detectNet("ssd-mobilenet-v2", threshold=0.5)
net = detectNet("peoplenet-pruned", threshold=0.5)

camera = videoSource("/dev/video0")      # '/dev/video0' for V4L2
display = videoOutput("display://0") # 'my_video.mp4' for file

while display.IsStreaming():
    img = camera.Capture()

    if img is None: # capture timeout
        continue

    detections = net.Detect(img)
    
    display.Render(img)
    display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
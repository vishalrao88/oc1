from jetson_utils import videoOutput

pipeline = "v4l2src device=/dev/video0 ! video/x-h264,width=640,height=480,framerate=30/1 ! h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw(memory:NVMM),format=RGBA ! nvvidconv ! video/x-raw,width=640,height=480,format=BGRx ! videoconvert ! appsink"
output = videoOutput(pipeline)

# Start the pipeline
#output.open()
#output.open()
#output.render

function image = takeimage()

camlist = webcamlist();

for i=1:2
    if strcmp(camlist{i},'Logi C270 HD WebCam')
        cam = webcam(i);
    end
end

image = snapshot(cam);

end
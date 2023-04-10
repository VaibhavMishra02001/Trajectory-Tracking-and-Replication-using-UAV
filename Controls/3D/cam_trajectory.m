cam = webcam();

img = snapshot(cam);
frameSize = size(img);
frameCount = 0;
x = zeros(0,2);
count1 =1;
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
nPoints = 50; % Number of points to interpolate
x = []; % Initialize x-coordinates of detected objects
y = []; % Initialize y-coordinates of detected objects

while frameCount < 4000
    img = snapshot(cam);
    diff_img = imsubtract(img(:,:,3),rgb2gray(img));
    diff_img = medfilt2(diff_img,[3,3]);
    diff_img = imbinarize(diff_img,0.18);
    diff_img = bwareaopen(diff_img,1200);
    bw=bwlabel(diff_img,8);
    stats=regionprops(logical(bw),'BoundingBox','Centroid');
    for object=1:length(stats)
        bbox = stats(object).BoundingBox;
        bc = stats(object).Centroid;
        bboxPoints = bbox2points(bbox(1, :));
        bboxPolygon = reshape(bboxPoints', 1, []);
%         plot(bc(1),bc(2),'-m+');
        x(end+1) = round(bc(1));
        y(end+1) = round(bc(2));
        
        % Interpolate points with B-spline
 if length(x) >= nPoints
      t = 1:length(x);
      ts = linspace(1,length(x),nPoints);
      xs = spline(t,x,ts);
      ys = spline(t,y,ts);
      plot(xs, ys, '-m+');
       x = xs(end);
            y = ys(end);
 else
            plot(x, y, '-m+');
 end
        
%         a=text(bc(1)+15,bc(2),strcat('X: ',num2str(round(bc(1))),'  Y :',num2str(round(bc(2)))));
        img = insertShape(img, 'polygon', bboxPolygon, 'LineWidth', 3);
        img = insertMarker(img, bc, "plus", "Color", "red");
    end
    
    frameCount = frameCount + 1;
    step(videoPlayer, img);
end

clear cam;
release(videoPlayer);

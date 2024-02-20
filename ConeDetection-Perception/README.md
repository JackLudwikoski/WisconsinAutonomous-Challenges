# Wisconsin Autonomous Perception Challenge Submission

## Answer.png:

![alt text](answer.png)

## Solution Methodology
To draw lines through the cones, there were two challenges I had to solve:
1. Detect the cones in the image
2. Draw lines that pass through the detected cone locations

### Detecting the cones
I ended up using the OpenCV Cascade Classifier to detect cones in the image. This is an object detection tool that relies on a series of weak classification methods strung together to create a more powerful classifier. Similarly to deep learning networks, this classifier requires a dataset of images to train it on. I collected images containing cones from google images for positive training samples and a collection of random images without cones for negative training samples. None of the cones from the image provided in the challenge specification were included in the training dataset. I then used the opencv_annotation tool to label the positive samples with bounding boxes around the cones. I also used the opencv_createsamples tool to take these labeled positive samples and generate a training dataset of uniform size. With a properly formatted training dataset, I was then able to use the opencv_traincascade tool to train a cascade classifier model. Finally, I applied this classifier to the image in order to detect all of the cones.
### Drawing the lines
With all of the cone locations identified in the image, I then decided to perform a linear regression on the cone locations on the left half of the screen as well as those on the right half of the screen to obtain the two desired lines. I found the following closed form solution for linear regression online and used that to compute the slope and y-intercept:

**Closed-Form Linear Regression**
$$a = \cfrac{\sum_{i=1}^n (x_i - \bar{x}) (y_i-\bar{y})}{\sum_{i=1}^n (x_i - \bar{x})^2}$$
$$b = \bar{y} - a \bar{x}$$
- a: slope
- b: y-intercept

I then used these parameters to compute the coordinates for the endpoints of the lines and then used the cv2.line() tool to draw the lines on the image.

## Challenges
The primary challenges in developing this solution came during the training process of the cascade classifier. Initially, the classifier was not detecting anything. This indicated to me that my training dataset might not be large enough for the classifier to understand what constitutes a cone. I ended up increasing from around 10 samples to 79 samples. After making this modification, I still was not detecting anything in the image. At this point in time, I did not have a great understanding of how cascade classifiers worked, so I decided to dive into the details. I discovered that I wasn't training the model with nearly enough stages. The power of the cascade classifier only comes when you utilize many stages because each stage only contains a few weak classifiers. I increased the number of training stages from 3 to 20. Upon retraining with this update parameter, I began detecting the cones. Unfortunately, I was also detecting a lot of things that weren't cones. I increased the negative samples in hopes that the model would then learn more about what is not a cone. This helped some, but I was still getting some false positive detections. I then began to parse out some of the positive samples that I felt were not very similar to the cones in the provided image. This last change finally brought me to a successful model.

## Other Attempted Solution Strategies
### Thresholding
One approach I attempted was to capitalize on the color difference of the cones relative to their surroundings. I first filtered out any bright near-white regions of the image. I then isolated the red channel of the image and set a threshold below which I set all pixels to black and above which I set all pixels to white. This successfully highlighted all of the cones as shown in the image below.

![alt text](threshold.png)

I then struggled with finding an easy way to define the cone locations in this image, so I moved on. In hindsight, this method likely could have been easier if I had followed it through.

### Edge Detection
While exploring the available functionality in OpenCV, I played around with edge detection a bit. I spent a bit of time brainstorming how I might be able to use this to address the problem at hand but moved on after not too long because it seemed to lack the capabilities that were required for this task.

### Template Matching
The template matching tool within OpenCV was definitely capable of leading me to a solution. This tool took as input a template of an image and then searched a larger image for that exact same image or a very similar image. I could have simply created templates off of all of the cones in the image and then applied those templates to find the cones within the image, but that felt like cheating because I was using the target image to create the template for analyzing itself. This also would have led to a very inflexible model that could not be applied to other situations, so I quickly decided not to proceed with this approach and to seek a more generalized solution.

## Libraries Used:
- OpenCV
    - opencv_annotation tool for labeling 
    - opencv_createsamples tool for creating training dataset
    - opencv_traincascade for training the cascade classifier model
    - Python library for loading the image, applying the cascade classifier, and displaying results


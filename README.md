# Image_Stitcher
Stitching 2 or more images to each other to form a larger image is done through a transformation matrix that projects (Transforms) 1 image plane into the other plane. This transformation matrix is called the "Homography" matrix in the world of Computer Vission. You phone probably uses this technique when you shoot a panoramic scene (photo). This repository is my code and solution for an assignment that was given to us in the course of Computer vission at ELTE.
Note that some of the file paths are written statically. So if you are using this code, make sure to read through it if I am using a static file path and such. However, the Homography files don't contain such a thing. They are very independent.

# Files Structure:
<ol>
  <li> <code>main.cpp</code> : Here the task solutions takes place. This is a very dependent code to the given problem. Here we load the images from the directory. The task explanation is given there as well. And the macros are defining the structure of the program (From visualizing the feature points, to creating the stitched images using a certain homography). </li>  
  <li> <code>functions.h</code> : Contain all the help funcitons that might be used during the whole program. Such as loading images, saving images and such. It also contains the feature points struct, which uses OpenCV ORB feature matching to extract the feature points between 2 images.</li>
  <li><code>homography.h</code> : is an interface (abstract class) which all the different kind of homographies classes will inherit from. Each of the homography classes demonstrates a certain way of homography matrix estimation. The most robust one is the RANSAC Normalized Homography estimator class. Check the results in the directory with the corresponding name.</li>
  <li>.... </li>
</ol>


# Running the project:
<ol>
  <li> first of all make sure to have opencv installed on your machine. You can follow how to set up with Visual Studio 19 on the "Bouncing Ball" Game on my github account. Read the README.md file to understand how to setup opencv with Visual Studio.</li>
  <li> after setting up openCV, you can run the project normal. The project <code>main.cpp</code> is task dependent. Thereofre, it will load the files from the the "./res" directory. If you want a different behavior you neeed to change that static part in the code </li>
  <li> finally you can set the macros that you want to see. Define them by uncomment them and the results will output on the same directory of the project. You will see something like "Homography_n". </li>
</ol>


# Examples:
<h2> Non-Normalized Homography Estimator:</h2>
<h3> 2 desired images: </h3>
<div>
  ![Dev1_Image_w960_h600_fn1000](https://user-images.githubusercontent.com/48254077/146667285-415f2004-cdeb-457c-b8b0-2da1e6a1161f.jpg)
  ![Dev2_Image_w960_h600_fn1000](https://user-images.githubusercontent.com/48254077/146667290-d78a0ca2-e675-4dba-926e-7135bf1fd9de.jpg)
</div>
<h3> Stitched Image </h3>
<div>
  ![HOMOGRAPHY_0](https://user-images.githubusercontent.com/48254077/146667267-a5ea8510-e82b-4104-b5e5-2d49f61456f5.png)
</div>
<br>
<hr>
<br>
<h2> Normalized Homography Estimator:</h2>
<h3> 2 desired images: </h3>
<div>
  ![Dev1_Image_w960_h600_fn1000](https://user-images.githubusercontent.com/48254077/146667285-415f2004-cdeb-457c-b8b0-2da1e6a1161f.jpg)
  ![Dev2_Image_w960_h600_fn1000](https://user-images.githubusercontent.com/48254077/146667290-d78a0ca2-e675-4dba-926e-7135bf1fd9de.jpg)
</div>
<h3> Stitched Image </h3>
<div>
  ![HOMOGRAPHY_0](https://user-images.githubusercontent.com/48254077/146667314-c421b8e6-efe4-4656-b639-3afde027b666.png)
</div>
<br>
<hr>
<br>
<h2> RANSAC Normalized Homography Estimator:</h2>
<h3> 2 desired images: </h3>
<div>
  ![Dev1_Image_w960_h600_fn1000](https://user-images.githubusercontent.com/48254077/146667285-415f2004-cdeb-457c-b8b0-2da1e6a1161f.jpg)
  ![Dev2_Image_w960_h600_fn1000](https://user-images.githubusercontent.com/48254077/146667290-d78a0ca2-e675-4dba-926e-7135bf1fd9de.jpg)
</div>
<h3> 2 desired images: </h3>
<div>
  ![HOMOGRAPHY_0](https://user-images.githubusercontent.com/48254077/146667327-2d96be89-ebfa-4b0d-ae3a-fc5f7048b296.png)
</div>
<br>
<hr>
<br>




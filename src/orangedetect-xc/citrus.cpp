//
//  citrus.cpp
//  orangedetect-xc
//
//  Created by Merrick Campbell on 5/13/21.
//

#include "citrus.hpp"

//**//**//**//**//**//**//**//**//
//  CONSTRUCTORS & DESTRUCTORS  //
//**//**//**//**//**//**//**//**//

CitrusDetector::CitrusDetector(bool scaleInput){
    // Save whether or not to scale image
    m_scaleInput =  scaleInput;
    
    // Scale Filtering Parameters Accordingly
    if(m_scaleInput){
        m_depthFilterKernelSize = 3;
        m_morphFilterKernelSize = 3;
        m_clusterThresh2D = m_clusterThresh2D * m_scaleFactor;
        m_clusterThresh3D = m_clusterThresh3D * m_scaleFactor;
        m_circleLineSize = 2;
    }
};

//**//**//**//**//**//**//**//
// PUBLIC MEMBER FUNCTIONS  //
//**//**//**//**//**//**//**//

CitrusDetector::Citrus CitrusDetector::findFruit(cv::Mat& imgColor, cv::Mat& imgDepth, CitrusDetector::citrusType fruitType, bool visualize){
    
    // Start Clock
    auto timeStart = std::chrono::system_clock::now();
    
    // Scale Image Before Processing
    if(m_scaleInput){
        cv::resize(imgDepth, imgDepth, cv::Size(), m_scaleFactor, m_scaleFactor, cv::INTER_AREA);
        cv::resize(imgColor, imgColor, cv::Size(), m_scaleFactor, m_scaleFactor, cv::INTER_AREA);
    }
    m_imgSize = imgColor.size();

    // Create a copy of the original image
    cv::Mat imgOrig = imgColor.clone();
    
    // Distance Filtering
    cv::Mat imgDistFiltered = removeBackground(imgColor, imgDepth,-1,-1,10,false);
    
    // Color Filtering
    int color = getTargetColorFromFruitType(fruitType);
    cv::Mat imgColorFiltered = colorFilter(imgDistFiltered, color,m_fruitColorRange);
    
    // Euclidean Clustering
    cv::Mat3b imgClustered = cv::Mat3b::zeros(imgColor.rows,imgColor.cols);
    std::vector<std::vector<cv::Point>> contours = clusterFruits(imgColorFiltered,imgDepth,imgClustered,CLUSTER_2D);
    
    // Circle Fitting
    m_foundFruits = fitCirclesToFruit(contours,true);
    if(filterFoundFruit(m_foundFruits, m_fruitMinRadius)){

    };
    
    // RANSAC
    
    // Correlation
    
    // End Clock
    auto timeEnd = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = timeEnd - timeStart;
    m_timeToRunMS = int(elapsed_seconds.count()*1000); //Convert to MS

    
    // Visualize
    if (visualize){
        // Draw Fruits on Images
        drawFruitCircles(imgColor,m_foundFruits);
//        drawFruitCircles(imgColorFiltered, m_foundFruits);

        // Create Combo Image to Show
        cv::Mat imgVis,imgText;
        
        cv::Mat col1,col2,col3,col12,col123;
        cv::vconcat(imgOrig, imgColor, col1);
        cv::vconcat(imgDepth, imgClustered, col2);
        cv::vconcat(imgDistFiltered, imgColorFiltered, col3);
        cv::hconcat(col1, col2, col12);
        cv::hconcat(col12, col3, col123);
        
        
        // Add Text Bar
        cv::Mat textBarTop = createTextBar(-1, -1, cv::Size(960,90),true);
        cv::Mat textBarBot = createTextBar(int(m_foundFruits.centers.size()), m_timeToRunMS, cv::Size(960,90),false);
        cv::vconcat(textBarTop, col123, imgText);
        cv::vconcat(imgText, textBarBot, imgVis);
        
        // Display
        cv::imshow("Detected Fruit",imgVis);
        
        // Record
        saveFrame(imgVis, "proc", m_frameNum);
    }
    
    //Increment Frame Number
    m_frameNum++;
    
    return m_foundFruits;
}

void CitrusDetector::setSaveFilePath(std::string path){m_savePath = path;}
void CitrusDetector::startRecord(){m_record = true;}
void CitrusDetector::stopRecord(){m_record = false;}

//**//**//**//**//**//**//**//
// PRIVATE MEMBER FUNCTIONS //
//**//**//**//**//**//**//**//

cv::Mat CitrusDetector::removeBackground(cv::Mat& imgColor, cv::Mat& imgDepth, int kernelSize, int numItr, int threshBGD, bool useGrabCut){
    
    // Create Mat Containers
    cv::Mat imgBW, imgFGD;
    
    // Create 1 Channel Greyscale Image
    cv::cvtColor(imgDepth, imgBW, cv::COLOR_BGR2GRAY);
    
    // Create Mask for Image
    cv::Mat imgMask(imgBW.size(), CV_8UC1);
    
    // Decide Which Segmentation Algorithm to Use
    if(useGrabCut){
        // Create Foreground and Background Mask Regions
        cv::Mat imgMaskFGD = createMaskTDE(imgBW, 180, kernelSize);
        imgBW.setTo(255,imgBW == 0); // RS treats 0 as special, it is not near the camera!
        cv::Mat imgMaskBGD = createMaskTDE(imgBW, 90, kernelSize);
        
        // Create Mask with guesses for GrabCut
        imgMask.setTo(cv::Scalar::all(cv::GC_BGD)); // Set "background" as default guess
        imgMask.setTo(cv::GC_PR_BGD, imgMaskBGD == 0); // Relax this to "probably background" for pixels outside "far" region
        imgMask.setTo(cv::GC_FGD, imgMaskFGD == 255); // Set pixels within the "near" region to "foreground"
        
        // Apply the GrabCut Algorithm
        cv::Mat m_imgBackground;
        cv::Mat m_imgForeground;
        cv::grabCut(imgColor, imgMask, cv::Rect(), m_imgBackground, m_imgBackground,numItr,cv::GC_INIT_WITH_MASK);
        
        // Create New Images
        imgFGD = cv::Mat::zeros(imgColor.rows, imgColor.cols, CV_8UC3);
        imgColor.copyTo(imgFGD,(imgMask == cv::GC_FGD) | (imgMask == cv::GC_PR_FGD));
        
    } else { // Use Simple Threshold Filter
        // Perform an Opening Operation (Erode, then Dilate)
        cv::Mat imgBlur, imgErode, imgDilate;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(m_depthFilterKernelSize,m_depthFilterKernelSize));
        
        // Smooth
        cv::GaussianBlur(imgBW, imgBlur, cv::Size(m_depthFilterKernelSize,m_depthFilterKernelSize), 0.5);
        
        // Erode
        cv::erode(imgBlur, imgErode, kernel,cv::Point(-1, -1),m_depthNumOpeningItr);
        
        // Dilate
        cv::dilate(imgErode, imgDilate, kernel,cv::Point(-1, -1),m_depthNumOpeningItr);
        
        // Apply Mask
        imgFGD = cv::Mat::zeros(imgColor.rows, imgColor.cols, CV_8UC3);
        imgColor.copyTo(imgFGD,(imgDilate > PIX_BLACK+threshBGD) );
    }
    
    return imgFGD;
}

cv::Mat CitrusDetector::createMaskTDE(cv::Mat& imgIn, int thresh, int edSize){
    // Create Structuring Element
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*edSize+1,2*edSize+1));
    
    // Threshold, Dilate, & Erode
    cv::Mat imgOut;
    cv::threshold(imgIn, imgOut, thresh, 255, cv::THRESH_BINARY);
    cv::dilate(imgOut, imgOut, kernel);
    cv::erode(imgOut, imgOut, kernel);
    
    return imgOut;
}

cv::Mat CitrusDetector::colorFilter(cv::Mat& img, int targetColor, int targetRange){
    // Create HSV image
    cv::Mat imgHSV;
    cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);
    
    // Get the H channel
    std::vector<cv::Mat> channelsHSV(3);
    cv::split(imgHSV, channelsHSV);
    cv::Mat hue = channelsHSV[0]; // H is the first channel
    
    // Get the G channel
    std::vector<cv::Mat> channelsBGR(3);
    cv::split(img, channelsBGR);
    cv::Mat green = channelsBGR[1]; // G is the second channel
    
    // Create Mask
    cv::Mat imgMask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
    imgMask.setTo(PIX_WHITE,(hue > targetColor - targetRange) & (hue < targetColor + targetRange)); // Select Citrus Color Pixels from Hue Space
    imgMask.setTo(PIX_BLACK,(green > 230)); // Remove Green Leaf Traces
    
    // Open Mask (Smooth, Erode, Dilate)
    imgMask = morphOpen(imgMask, 3);
    
    // Throwaway values outside range
    cv::Mat imgFruit = cv::Mat::zeros(img.rows,img.cols, CV_8UC3);
    img.copyTo(imgFruit,imgMask);
    return imgFruit;
}

int CitrusDetector::getTargetColorFromFruitType(citrusType type){
    switch (type) {
        case CITRUS_ORANGE:
            return m_fruitColorOrange;
            break;
        case CITRUS_GRAPEFRUIT:
            return m_fruitColorGrapefruit;
            break;
        default:
            std::cerr << "Invalid Citrus Type in getTargetColorFromFruitType" << std::endl;
            return 90; // Value Halfway between [0-179]
            break;
    }
}


cv::Mat CitrusDetector::morphOpen(cv::Mat& img, int numItr){
    // Create Mats
    cv::Mat imgBlur, imgErode, imgDilate;
    
    // Get Kernel
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(m_morphFilterKernelSize,m_morphFilterKernelSize));
    
    // Smooth
    cv::GaussianBlur(img, imgBlur, cv::Size(m_morphFilterKernelSize,m_morphFilterKernelSize), 0.5);
    
    // Erode
    cv::erode(imgBlur, imgErode, kernel,cv::Point(-1, -1),numItr);
    
    // Dilate
    cv::dilate(imgErode, imgDilate, kernel,cv::Point(-1, -1),numItr);
    
    return imgDilate;
}

std::vector<std::vector<cv::Point>> CitrusDetector::clusterFruits(cv::Mat& imgColor, cv::Mat& imgDepth, cv::Mat3b& imgCluster, CitrusDetector::citrusCluster clusterMethod){
    
    //Ref: https://docs.opencv.org/4.5.2/d5/d38/group__core__cluster.html#ga2037c989e69b499c1aa271419f3a9b34 (Official Documentation)
    //Ref: https://pcl.readthedocs.io/en/latest/cluster_extraction.html (Original Paper uses this!)
    //Ref: https://stackoverflow.com/questions/33825249/opencv-euclidean-clustering-vs-findcontours (Good use of lambda expression)
    
    // Determine whether to perform a 2D or 3D cluster
    bool use3D = (clusterMethod==CLUSTER_3D);
    
    // Create a mask of the color regions
    cv::Mat mask;
    cv::cvtColor(imgColor,mask,cv::COLOR_BGR2GRAY);

#if DEBUG_CLUSTER
    cv::imshow("ColorMask",mask);
    cv::imshow("Depth3D",imgDepth);
    cv::waitKey(0);
#endif
    
    // Get all non black points in the image mask
    std::vector<cv::Point> pts;
    cv::findNonZero(mask, pts);
    
    // Build X,Y,Z Points for filtering
    std::vector<cv::Point3i> pts3d;
    for(int i = 0; i < pts.size(); i++){
        // Get the depth coordinate from the x and y;
        int x = pts.at(i).x;
        int y = pts.at(i).y;
        int z = (use3D ? imgDepth.at<uchar>(x,y) : 0); //Set to 0 if 2D

#if DEBUG_CLUSTER
        std::cout<< x << "," << y << "," << z << std::endl;
#endif

        // Add the 3D Point to the vector
        pts3d.push_back(cv::Point3i(x,y,z));

    }
    
    // Define the radius tolerance
    int th_distance = (use3D ? m_clusterThresh3D : m_clusterThresh2D); // radius tolerance
    
    // Apply partition
    // All pixels within the radius tolerance distance will belong to the same label
    std::vector<int> labels;
    int th2 = th_distance * th_distance; // Avoid square root
    int n_labels = cv::partition(pts3d, labels, [th2](const cv::Point3i& lhs, const cv::Point3i& rhs) {
        return ((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y) + (lhs.z - rhs.z)*(lhs.z - rhs.z)) < th2;
    });
    
    // Save all points in the same class in a vector (one for each class)
    std::vector<std::vector<cv::Point>> contours(n_labels);
    for (int i = 0; i < pts.size(); ++i)
    {
        contours[labels[i]].push_back(pts[i]);
    }
        
    // Build a vector of random color, one for each class (label)
    std::vector<cv::Vec3b> colors;
    for (int i = 0; i < n_labels; ++i)
    {
        colors.push_back(cv::Vec3b(rand() & 255, rand() & 255, rand() & 255));
    }
    
    // Apply Clusters to Labels
    for (int i = 0; i < pts.size(); ++i)
    {
        imgCluster(pts[i]) = colors[labels[i]];
    }
    
    // Return the found contours
    return contours;
}

CitrusDetector::Citrus CitrusDetector::fitCirclesToFruit(std::vector<std::vector<cv::Point>> contours, bool useHough){
    
    //Ref: https://docs.opencv.org/4.5.2/da/d0c/tutorial_bounding_rects_circles.html
    //Ref: https://docs.opencv.org/4.5.2/d4/d70/tutorial_hough_circle.html
    
    //TODO: experiment with more advanced fitting algorithms, RANSAC?
    // Declare Variables
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Point2f>centers( contours.size() );
    std::vector<float>radius( contours.size() );
    std::vector<cv::Vec3f> circles;
    
    
    // Loop through the contours
    for( int i = 0; i < contours.size(); i++ )
    {
        if (useHough){
            // Draw the contour on the image
            cv::Mat mask = cv::Mat::zeros(m_imgSize,CV_8UC1);
            cv::drawContours(mask, contours, i, cv::Scalar(PIX_WHITE,PIX_WHITE,PIX_WHITE));
            
            // Fit circle with the Hough Transform
            cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT_ALT, .25,1,300,0.25,m_fruitMinRadius,0);
            
            // Add to the array
            if(circles.size() > 0){
                // Initialize points to sum
                int x=0;
                int y=0;
                int r=0;
                
                // Calculate the average value
                for( int j=0; j < circles.size() ; j++){
                    // Get center and radius
                    cv::Point center(cvRound(circles[j][0]), cvRound(circles[j][1]));
                    int radius = cvRound(circles[j][2]);
                    
                    // Add to sum
                    x += center.x;
                    y += center.y;
                    r += radius;

                }

                // Add to output candidates
                centers[i] = cv::Point(int(x/circles.size()),int(y/circles.size()));
                radius[i] = int(r/circles.size());
                
            }

#if DEBUG_HOUGH
            std::cerr << "Hough Detected " << circles.size() << " circles" << std::endl;
            cv::Mat colorMask;
            cv::cvtColor(mask, colorMask, cv::COLOR_GRAY2BGR);
            for( int i = 0; i < circles.size(); i++ )
            {
                 cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                 int radius = cvRound(circles[i][2]);
                 // draw the circle center
                 cv::circle( colorMask, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
                 // draw the circle outline
                 cv::circle( colorMask, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
            }
            cv::imshow("Hough",colorMask);
            cv::waitKey(0);
#endif
            
        } else { // Fit min enclosing circle to the contours

            // Use a polygon approximation for the contour blobs
            cv::approxPolyDP( contours[i], contours_poly[i], 3, true );
            
            // Find the minimum enclosing circle
            cv::minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
        }
    
    }
    
    // Prepare a citrus struct
    Citrus fruitCircles;
    fruitCircles.centers = centers;
    fruitCircles.radius = radius;
    
    // Return the citrus struct
    return fruitCircles;
}

bool CitrusDetector::filterFoundFruit(CitrusDetector::Citrus& fruit, float radiusThresh){
    // Determine the input length
    int lengthIn = int(fruit.centers.size());
    
    // Iterate through the found fruit
    for(int i = lengthIn-1; i >= 0; i--){
        std::cout << fruit.radius[i] << std::endl;
        // Remove fruits that are too small (both RADIUS and CENTERS!)
        float radius = fruit.radius.at(i);
        
        if(radius < radiusThresh){

            fruit.radius.erase(fruit.radius.begin() + i);
            fruit.centers.erase(fruit.centers.begin() + i);
        }
    }
    
    // Determine the output length
    int lengthOut = int(fruit.centers.size());
    
    // Return [T] if the length has changed
    return (lengthIn != lengthOut);
}

void CitrusDetector::drawFruitCircles(cv::Mat& img, CitrusDetector::Citrus fruit){
        
    // Iterate through the fruit
    for( int i = 0; i< fruit.centers.size(); i++ )
    {
        // Draw the Circles
        cv::circle( img, fruit.centers[i], (int)fruit.radius[i], m_circleColor, m_circleLineSize);
        cv::drawMarker(img, fruit.centers[i], m_circleColor);
    }
    
}

cv::Mat CitrusDetector::createTextBar(int numCitrusFound, int timeMS, cv::Size size, bool top){
    cv::Mat textBar = cv::Mat::zeros(size,CV_8UC3);
    
    if (top){
        cv::putText(textBar, "Citrus Detection", cv::Point(340,50), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255,255,255));

    } else { // Bottom Bar
        std::string count = "Citrus Count: " + std::to_string(numCitrusFound);
        std::string rate = "Compute Time (MS): " + std::to_string(timeMS);
        cv::putText(textBar, count, cv::Point(20,50), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255,255,255));
        cv::putText(textBar, rate, cv::Point(500,50), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255,255,255));
    }
    
    
    return textBar;
}



bool CitrusDetector::saveFrame(cv::Mat& img, std::string frameName, int frameNum){
    if(m_record){
        // Zero-Pad the Frame Counter
        std::ostringstream ss;
        ss << std::setw(5) << std::setfill('0') << frameNum;
        std::string str = ss.str();
    
        // Save the Images
        std::string fileOut = m_savePath + frameName + str + ".png";
        cv::imwrite(fileOut, img);
        return true;
    } else {
        return false;
    }
}

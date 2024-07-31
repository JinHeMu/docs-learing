# OpenCV

## 读取图片和视频

```cpp
	string path = "Resources/test.png";
	Mat img = imread(path);
    imshow("Image", img);
    waitKey(0);
```

```cpp
	string path = "Resources/test_video.mp4";
    VideoCapture cap(path);
//	VideoCapture cap(0);读取设备摄像头
    Mat img;

    while(true)
    {
        cap.read(img);
        imshow("image", img);
        waitKey(1);

    }
```

## 颜色转换

`cvtColor`是OpenCV库中的一个函数，用于进行颜色空间转换。具体而言，`cvtColor`表示"convert color"，它的目的是将一个图像从一个颜色空间转换为另一个颜色空间。

在你提供的代码中，`cvtColor`被用来将一个彩色图像转换为灰度图像。函数的调用如下：

```cpp
cvtColor(img, imgGray, COLOR_BGR2GRAY);
```

这里的参数解释如下：
- `img`：原始彩色图像
- `imgGray`：转换后的灰度图像
- `COLOR_BGR2GRAY`：颜色转换代码，表示将BGR格式的彩色图像转换为灰度图像。

这个函数在图像处理中经常用于简化问题，因为灰度图像只包含亮度信息，而不包含颜色信息，因此可以减少处理的复杂性。

如果你对颜色空间转换或OpenCV有更多的疑问，请随时问我。

## 高斯模糊

高斯模糊是一种图像处理技术，其原理基于数学中的高斯分布（正态分布）函数。该模糊方法用于降低图像中噪声的影响、平滑图像以及减小图像中细节的尺度。

高斯模糊的基本原理是在图像的每个像素周围应用一个二维高斯核（Gaussian Kernel）来加权平均邻近像素的值。这个二维高斯核的形状由高斯分布函数决定，距离核心像素越远的像素会被赋予较小的权重，而距离较近的像素则被赋予较大的权重。

高斯核的一维形式如下：

$$
G(x) = \frac{1}{\sqrt{2\pi\sigma^2}} \exp\left(-\frac{x^2}{2\sigma^2}\right)
$$
其中，\(x\) 是距离核心的偏移，\($\sigma$\) 是高斯核的标准差，决定了核的宽度。对于二维高斯核，可以通过在两个方向上分别应用一维高斯核来得到。

应用高斯模糊时，图像中的每个像素都通过与高斯核的卷积来计算新的像素值。这样做的效果是，图像中的高频噪声被抑制，平滑度增加，图像中的细节减弱。

在OpenCV中，可以使用`GaussianBlur`函数来实现高斯模糊。以下是一个简化的示例：

```cpp
#include <opencv2/opencv.hpp>

int main() {
    cv::Mat inputImage = cv::imread("input.jpg");

    if (inputImage.empty()) {
        std::cerr << "Unable to load image!" << std::endl;
        return -1;
    }

    cv::Mat blurredImage;
    cv::GaussianBlur(inputImage, blurredImage, cv::Size(5, 5), 0); // 5x5的高斯核，标准差为0表示自动计算

    cv::imshow("Original Image", inputImage);
    cv::imshow("Blurred Image", blurredImage);
    cv::waitKey(0);

    return 0;
}
```

这个示例中，`cv::GaussianBlur`函数接受输入图像、输出图像以及高斯核的大小和标准差等参数。

高斯模糊是图像处理中常用的一种模糊滤波方法。它采用了高斯函数的权重来平滑图像中的像素值，从而降低图像中的噪声并模糊细节。高斯模糊的基本思想是对图像中的每个像素点施加一个以该点为中心的二维高斯函数，然后将所有像素的值按照高斯函数的权重进行加权平均。

使用高斯模糊有几个常见的目的：

1. **噪声消除：** 高斯模糊可以有效地去除图像中的高频噪声，使图像更加平滑。

2. **边缘保留：** 虽然是模糊操作，但高斯模糊在一定程度上能够保留图像的边缘信息，不像一些其他模糊方法那样会导致边缘模糊。

3. **图像预处理：** 在某些图像处理任务中，如目标检测或图像识别，常常会在处理之前对图像进行高斯模糊，以减少细节并简化后续的处理过程。

高斯模糊的强度取决于所使用的高斯核的标准差，标准差越大，模糊效果越明显。在图像处理软件或库中，通常可以通过指定标准差来控制高斯模糊的程度。

## Canny边缘检测

Canny边缘检测是一种常用的图像处理技术，用于检测图像中的边缘。该算法由John Canny于1986年提出，具有较好的边缘检测效果和抗噪声能力。

Canny边缘检测的主要步骤包括：

1. **降噪：** 使用高斯滤波对图像进行平滑，以减小噪声的影响。这有助于提高后续边缘检测的准确性。

2. **计算图像梯度：** 使用Sobel等算子计算图像中每个像素点的梯度和方向。梯度表示图像中像素值变化最快的方向。

3. **非极大值抑制：** 对梯度图进行扫描，保留梯度方向上的局部极大值点，以细化边缘。

4. **滞后阈值处理：** 设置两个阈值，高阈值和低阈值。将梯度图中的像素分为强边缘、弱边缘和非边缘三类。保留强边缘，然后通过弱边缘的连接来形成完整的边缘。

Canny边缘检测在图像处理和计算机视觉中广泛应用，因为它能够有效地捕捉图像中的显著特征，例如物体边界。

在OpenCV中，你可以使用`cv::Canny`函数来实现Canny边缘检测。以下是一个简化的示例：

```cpp
#include <opencv2/opencv.hpp>

int main() {
    cv::Mat inputImage = cv::imread("input.jpg", cv::IMREAD_GRAYSCALE);

    if (inputImage.empty()) {
        std::cerr << "Unable to load image!" << std::endl;
        return -1;
    }

    cv::Mat edges;
    cv::Canny(inputImage, edges, 50, 150); // 设置高低阈值

    cv::imshow("Original Image", inputImage);
    cv::imshow("Canny Edges", edges);
    cv::waitKey(0);

    return 0;
}
```

在这个示例中，`cv::Canny`函数接受输入图像、输出图像以及高低阈值等参数。

## 膨胀和腐蚀

在OpenCV中，膨胀和腐蚀的操作可以通过 `cv::dilate` 和 `cv::erode` 函数来实现。

以下是一个简单的示例，展示了如何在OpenCV中使用这两个函数：

```cpp
#include <opencv2/opencv.hpp>

int main() {
    cv::Mat inputImage = cv::imread("input.jpg", cv::IMREAD_GRAYSCALE);

    if (inputImage.empty()) {
        std::cerr << "Unable to load image!" << std::endl;
        return -1;
    }

    // 定义结构元素（kernel）
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    // 膨胀操作
    cv::Mat dilatedImage;
    cv::dilate(inputImage, dilatedImage, kernel);

    // 腐蚀操作
    cv::Mat erodedImage;
    cv::erode(inputImage, erodedImage, kernel);

    cv::imshow("Original Image", inputImage);
    cv::imshow("Dilated Image", dilatedImage);
    cv::imshow("Eroded Image", erodedImage);
    cv::waitKey(0);

    return 0;
}
```

在这个示例中，`cv::getStructuringElement` 用于创建一个矩形形状的结构元素，然后 `cv::dilate` 和 `cv::erode` 函数分别对输入图像进行膨胀和腐蚀操作。你可以调整结构元素的大小和形状，以及调整膨胀和腐蚀的程度来满足特定的需求。

## 裁剪和放缩

在OpenCV中，裁剪（截取图像的一部分）和放缩（改变图像的大小）是常见的图像处理操作。以下是使用OpenCV实现裁剪和放缩的简单示例：

### 裁剪图像：

```cpp
#include <opencv2/opencv.hpp>

int main() {
    cv::Mat inputImage = cv::imread("input.jpg");

    if (inputImage.empty()) {
        std::cerr << "Unable to load image!" << std::endl;
        return -1;
    }

    // 定义裁剪区域
    cv::Rect roi(100, 50, 200, 150); // (x, y, width, height)

    // 裁剪图像
    cv::Mat croppedImage = inputImage(roi);

    cv::imshow("Original Image", inputImage);
    cv::imshow("Cropped Image", croppedImage);
    cv::waitKey(0);

    return 0;
}
```

在这个示例中，`cv::Rect` 定义了一个矩形区域，指定了裁剪的区域（左上角的坐标和宽高）。通过将这个矩形区域传递给输入图像，可以获得裁剪后的图像。

### 放缩图像：

```cpp
#include <opencv2/opencv.hpp>

int main() {
    cv::Mat inputImage = cv::imread("input.jpg");

    if (inputImage.empty()) {
        std::cerr << "Unable to load image!" << std::endl;
        return -1;
    }

    // 定义目标大小
    cv::Size newSize(300, 200); // (width, height)

    // 放缩图像
    cv::Mat resizedImage;
    cv::resize(inputImage, resizedImage, newSize);

    cv::imshow("Original Image", inputImage);
    cv::imshow("Resized Image", resizedImage);
    cv::waitKey(0);

    return 0;
}
```

在这个示例中，`cv::resize` 函数用于调整图像的大小。你可以通过指定目标大小（宽和高）或缩放因子来进行放缩。在上述示例中，通过 `cv::Size` 定义了一个新的大小。如果你想按比例缩放，你可以通过计算缩放因子来实现。

这些示例提供了基本的裁剪和放缩操作，你可以根据具体的需求调整裁剪区域、目标大小或其他参数。

## 绘制形状和文字

```c++
int main()
{
    // Blank Image
    Mat  img(512, 512, CV_8UC3, Scalar(255, 255, 255));

    circle(img, Point(256,256), 155, Scalar(0, 69, 255), -1);
    rectangle(img, Point(130, 226), Point(382, 286), Scalar(255, 255, 255), 10);
    line(img, Point(130,296), Point(382,450), Scalar(255, 255, 255), 10);

    putText(img, "jhm is handsome boy", Point(137,450), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 69, 255));

    imshow("img", img);
    waitKey(0);
    return 0;
}
```

## 透视变换

这段代码是使用OpenCV进行透视变换（Perspective Transformation）的操作。透视变换可以用于将图像中的四边形区域映射到另一视图中的任意四边形，从而实现图像的透视效果。

```cpp
cv::Mat matrix = cv::getPerspectiveTransform(src, dst);
cv::warpPerspective(img, imgWarp, matrix, cv::Point(w, h));
```

这里的关键函数有两个：

1. **getPerspectiveTransform：** 该函数用于计算透视变换矩阵。它接受两个参数：`src`是原图中的四个点的坐标，`dst`是这四个点在输出图像中对应的坐标。

2. **warpPerspective：** 该函数用于应用透视变换。它接受四个参数：`img`是输入图像，`imgWarp`是输出图像，`matrix`是透视变换矩阵，`cv::Point(w, h)`是输出图像的大小。

在这段代码中，`src`和`dst`分别是原图中和输出图中的四个点的坐标，`matrix`是计算得到的透视变换矩阵。然后，通过 `warpPerspective` 函数将原图 `img` 进行透视变换，结果存储在 `imgWarp` 中。

这通常用于校正图像中的透视失真，例如从一个倾斜的角度拍摄的文档照片，将其转换为正面视图。

## HSV

HSV代表色调（Hue）、饱和度（Saturation）和亮度（Value），是一种用于描述颜色的颜色模型。HSV模型是将颜色信息从RGB模型中分离出来的一种方式，使得人们更容易理解和调整颜色。

这三个分量的含义如下：

1. **色调（Hue）：** 表示颜色的种类。它是一个0到360度的值，对应于颜色在圆环上的位置。0度和360度表示红色，120度表示绿色，240度表示蓝色，以此类推。

2. **饱和度（Saturation）：** 表示颜色的纯度或深浅程度。它是一个百分比值，从0%（灰色，无色彩）到100%（最鲜艳的颜色）。

3. **亮度（Value）：** 表示颜色的明暗程度。它也是一个百分比值，从0%（黑色）到100%（白色）。

HSV模型提供了一种更直观的方式来描述颜色，尤其在图形设计和图像处理领域经常被使用。在OpenCV等图像处理库中，HSV模型常常用于调整图像的颜色，进行颜色分割等操作。

## 跟踪栏

```cpp
namedWindow("Trackbars", (640, 200));
createTrackbar("Hue min", "Trackbars", &hmin, 179);
```

可以创建一个跟踪栏来实时调节参数值,以获得我们想要的色块

## 轮廓检测

### 预处理

二值化,高斯模糊,Canny边缘检测,膨胀

其中高斯模糊用来减小噪声,膨胀可以用来加大边缘检测效果

```cpp
cvtColor(img, imgGray, COLOR_BGR2GRAY);
GaussianBlur(imgGray, imgBlur, Size(3, 3), 3, 0);
Canny(imgBlur, imgCanny, 25, 75);

Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
dilate(imgCanny, imgDil, kernel);
```

### 查找轮廓和绘制

```cpp
vector<vector<Point>> contours;
vector<Vec4i> hierarchy;

findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
drawContours(img, contours, -1, Scalar(255, 0, 255), 2);
```

这段代码使用了OpenCV库进行轮廓检测和绘制操作。让我们逐行解释：

1. `findContours` 函数用于在二值化图像 `imgDil` 上查找轮廓，并将结果存储在 `contours` 中，层次结构信息存储在 `hierarchy` 中。具体的参数含义如下：
   - `imgDil`: 输入的二值化图像。
   - `contours`: 用于存储检测到的轮廓的向量。
   - `hierarchy`: 用于存储轮廓层次结构信息的向量。
   - `RETR_EXTERNAL`: 轮廓检索模式，表示仅检测外部轮廓。
   - `CHAIN_APPROX_SIMPLE`: 轮廓近似方法，表示使用简化的近似方法。
   
2. `drawContours` 函数用于在图像 `img` 上绘制检测到的轮廓。具体的参数含义如下：

   - `img`: 输出的图像，轮廓将在这个图像上被绘制。
   - `contours`: 存储轮廓的向量。
   - `-1`: 表示绘制所有检测到的轮廓。
   - `Scalar(255, 0, 255)`: 绘制轮廓的颜色，这里是紫色（RGB表示）。
   - `2`: 绘制轮廓的线宽度。

这两行代码一起完成了轮廓检测和绘制的操作。检测到的轮廓存储在 `contours` 中，可以在后续的代码中使用这些轮廓进行进一步的图像处理或分析。而绘制的结果则存储在 `img` 中，你可以根据需要保存或显示这个图像。

```cpp
void getContours(const Mat& imgDil_V, const Mat& img)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(imgDil_V, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    string objectType;

    for (int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        cout << area << endl;

        vector<vector<Point>> conPoly(contours.size());
        vector<Rect> boundRect(contours.size());

        if (area > 1000 )
        {
            double peri = arcLength(contours[i], true);
            approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
            drawContours(img, conPoly, i, Scalar(255, 0, 255), 2);
            cout << conPoly[i].size() << endl;
            boundRect[i] = boundingRect(conPoly[i]);
            rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 5);

            int objCor = (int)conPoly[i].size();

            if(objCor == 3){ objectType = "Tri"; }
            if(objCor == 4)
            {
                float aspRatio = (float)boundRect[i].width / (float)boundRect[i].height;
                if(aspRatio > 0.95 && aspRatio < 1.05)
                {
                    objectType = "Square";
                }
                else
                {
                    objectType = "Rect";
                }

            }
            if(objCor > 4){ objectType = "Circle"; }

            putText(img, objectType, {boundRect[i].x, boundRect[i].y - 5}, FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 69, 255));
        }
    }
}
```

## 面部检测

```cpp
int main()
{
//    string path = "Resources/test.png";
//    Mat img = imread(path);


    VideoCapture cap(0);
    Mat img;
    
    while(true)
    {
        cap.read(img);
        imshow("image", img);

        CascadeClassifier faceCascade;
        faceCascade.load("Resources/haarcascade_frontalface_default.xml");

        if(faceCascade.empty()){ cout << "XML file not loaded" << endl; }

        vector<Rect> faces;
        faceCascade.detectMultiScale(img, faces, 1.1, 10);

        for (int i = 0; i < faces.size(); i++) {
            Mat imgCrop = img(faces[i]);
            imshow(to_string(i), imgCrop);
            rectangle(img, faces[i].tl(), faces[i].br(), Scalar(255, 0 ,255), 3);
        }
        imshow("img", img);
        waitKey(1);
    }
    return 0;
}
```

`CascadeClassifier` 是 OpenCV 库中用于对象检测的一个类。它主要用于实现基于 Haar 特征的级联分类器（Cascade Classifier）的目标检测方法。

Haar 特征是一种基于图像局部特征的检测方法，通过使用弱分类器组成的级联结构来实现高效的对象检测。Cascade Classifier 通过级联的方式，在不同阶段进行粗糙的筛选，将可能包含对象的区域传递给下一级，最终得到一个相对准确的检测结果。

在 OpenCV 中，`CascadeClassifier` 可以用于加载已经训练好的 Haar 特征分类器模型，并用于检测图像中是否包含特定类型的对象，比如人脸、眼睛等。


在OpenCV中，`detectMultiScale` 函数是用于在图像中检测对象的方法。具体来说，它是基于级联分类器（通常是Haar级联）的目标检测方法。

下面是你提供的 `detectMultiScale` 函数的参数解释：

- `img`: 输入的图像。这是需要检测对象的源图像。
- `faces`: 一个 `std::vector<cv::Rect>` 类型的变量，用于存储检测到的对象的矩形区域。在你的例子中，这是用于存储检测到的人脸矩形的容器。
- `1.1`: 表示在图像中搜索对象时每次图像缩小的比例。这个值越大，检测速度越快，但可能会错过小目标。通常选择在1.01到1.3之间的值。
- `10`: 表示目标的最小邻居数，用于确定目标区域。较大的值会去掉一些重叠较多的检测框，通常选择在3到30之间的值。

这个函数会返回一个 `std::vector<cv::Rect>`，其中包含检测到的对象的矩形区域。在你的代码中，这个矩形区域表示检测到的人脸的位置。


from PySide6 import QtWidgets, QtCore, QtGui
import cv2 as cv
import os, time
from threading import Thread

# 屏蔽YOLO处理输出的调试信息
os.environ['YOLO_VERBOSE'] = 'False'
from ultralytics import YOLO

class MWindow(QtWidgets.QMainWindow):
    def __init__(self):

        super().__init__()

        # 设置界面
        self.setupUI()

        self.camBtn.clicked.connect(self.startCamera)
        self.videoBtn.clicked.connect(self.startVideoFile)
        self.stopBtn.clicked.connect(self.stop)

        # 定义定时器，用于控制显示摄像头视频的帧率
        self.timer_camera = QtCore.QTimer()
        # 定时到了，回调 self.show_camera
        self.timer_camera.timeout.connect(self.show_camera)

        # 加载 YOLO nano 模型，第一次比较耗时，要20秒左右
        self.model = YOLO('../../model/yolov8m.pt')

        # 要处理的视频帧图片队列，目前就放1帧图片
        self.frameToAnalyze = []

        # 启动处理视频帧独立线程
        Thread(target=self.frameAnalyzeThreadFunc,daemon=True).start()

        # 定义定时器，用于控制显示视频文件的帧率
        self.timer_videoFile = QtCore.QTimer()
        # 定时到了，回调 self.show_camera
        self.timer_videoFile.timeout.connect(self.show_videoFile)

        # 当前要播放的视频帧号
        self.vframeIdx = 0

        # cv.VideoCapture 实例
        self.cap = None

        self.stopFlag = False

    def setupUI(self):

        self.resize(1200, 800)

        self.setWindowTitle(' YOLO-PyQt Demo')

        # central Widget
        centralWidget = QtWidgets.QWidget(self)
        self.setCentralWidget(centralWidget)

        # central Widget 里面的 主 layout
        mainLayout = QtWidgets.QVBoxLayout(centralWidget)

        # 界面的上半部分 : 图形展示部分
        topLayout = QtWidgets.QHBoxLayout()
        self.label_ori_video = QtWidgets.QLabel(self)
        self.label_treated = QtWidgets.QLabel(self)
        self.label_ori_video.setFixedSize(520,400)
        self.label_treated.setFixedSize(520,400)
        # self.label_ori_video.setMinimumSize(520,400)
        # self.label_treated.setMinimumSize(520,400)
        self.label_ori_video.setStyleSheet('border:1px solid #D7E2F9;')
        self.label_treated.setStyleSheet('border:1px solid #D7E2F9;')

        topLayout.addWidget(self.label_ori_video)
        topLayout.addWidget(self.label_treated)

        mainLayout.addLayout(topLayout)

        # 界面下半部分： 输出框 和 按钮
        groupBox = QtWidgets.QGroupBox(self)

        bottomLayout =  QtWidgets.QHBoxLayout(groupBox)
        self.textLog = QtWidgets.QTextBrowser()
        bottomLayout.addWidget(self.textLog)

        mainLayout.addWidget(groupBox)

        btnLayout = QtWidgets.QVBoxLayout()
        self.videoBtn = QtWidgets.QPushButton('🎞️视频文件')
        self.camBtn   = QtWidgets.QPushButton('📹摄像头')
        self.stopBtn  = QtWidgets.QPushButton('🛑停止')
        btnLayout.addWidget(self.videoBtn)
        btnLayout.addWidget(self.camBtn)
        btnLayout.addWidget(self.stopBtn)
        bottomLayout.addLayout(btnLayout)


    def startCamera(self):

        # 参考 https://docs.opencv.org/3.4/dd/d43/tutorial_py_video_display.html

        # 在 windows上指定使用 cv.CAP_DSHOW 会让打开摄像头快很多，
        # 在 Linux/Mac上 指定 V4L, FFMPEG 或者 GSTREAMER
        self.cap = cv.VideoCapture(0, cv.CAP_DSHOW)
        if not self.cap.isOpened():
            print("1号摄像头不能打开")
            return

        if self.timer_camera.isActive() == False:  # 若定时器未启动
            self.timer_camera.start(30)
            self.stopFlag = False


    def show_camera(self):

        ret, frame = self.cap.read()  # 从视频流中读取
        if not ret:
            return

        # 把读到的帧的大小重新设置
        # frame = cv.resize(frame, (520, 400))

        self.setFrameToOriLabel(frame)

    def setFrameToOriLabel(self,frame):

        # 视频色彩转换回RGB，OpenCV images as BGR
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        qImage = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0],
                                 QtGui.QImage.Format_RGB888)  # 变成QImage形式
        # 往显示视频的Label里 显示QImage
        self.label_ori_video.setPixmap(QtGui.QPixmap.fromImage(qImage))

        # 如果当前没有处理任务
        if not self.frameToAnalyze:
            self.frameToAnalyze.append(frame)

    def frameAnalyzeThreadFunc(self):

        while True:
            if not self.frameToAnalyze:
                time.sleep(0.01)
                continue

            frame = self.frameToAnalyze.pop(0)

            results = self.model(frame)[0]

            img = results.plot(line_width=1)

            qImage = QtGui.QImage(img.data, img.shape[1], img.shape[0],
                                    QtGui.QImage.Format_RGB888)  # 变成QImage形式

            if self.stopFlag == False:
                self.label_treated.setPixmap(QtGui.QPixmap.fromImage(qImage))  # 往显示Label里 显示QImage

            # time.sleep(0.5)

    def stop(self, ):

        self.stopFlag = True      # 让 frameAnalyzeThreadFunc 不要再设置 label_treated
        self.timer_camera.stop()  # 关闭定时器
        self.timer_videoFile.stop()  # 关闭定时器

        if self.cap:
            self.cap.release()  # 释放视频流

        # 清空视频显示区域
        self.label_ori_video.clear()
        self.label_treated.clear()

        # # 延时500ms清除，有的定时器处理任务可能会在当前时间点后处理完最后一帧
        # QtCore.QTimer.singleShot(500, clearLabels)

    def startVideoFile(self):

        # 先关闭原来打开的
        self.stop()

        videoPath, _  = QtWidgets.QFileDialog.getOpenFileName(
            self,             # 父窗口对象
            "选择视频文件",        # 标题
            ".",               # 起始目录
            "图片类型 (*.mp4 *.avi)" # 选择类型过滤项，过滤内容在括号中
        )

        print('videoPath is', videoPath)
        if not videoPath:
            return


        self.cap = cv.VideoCapture(videoPath)
        if not self.cap.isOpened():
            print("打开文件失败")
            return


        self.timer_videoFile.start(30)
        self.stopFlag = False

        print("ok")


    def show_videoFile(self):

        # 选取视频帧位置，
        self.cap.set(cv.CAP_PROP_POS_FRAMES, self.vframeIdx)
        self.vframeIdx += 1
        ret, frame = self.cap.read()  # 从视频流中读取

        # 读取失败，应该是视频播放结束了
        if not ret:
            self.stop()
            return

        self.setFrameToOriLabel(frame)

app = QtWidgets.QApplication()
window = MWindow()
window.show()
app.exec()

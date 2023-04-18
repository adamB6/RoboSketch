# importing libraries
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys
import json

#Window class
class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        
        #Setting up Window
        self.setWindowTitle("RobotUI")
        self.setGeometry(100, 100, 800, 800)

        #Setting up background color
        self.image = QImage(self.size(), QImage.Format_RGB32)
        self.image.fill(Qt.white)

		#Flags
        self.drawing = False
        self.freeDrawing = False
        self.lineDrawing = False
        self.contLineDrawing = False
        
        #Setting up Default Brush
        self.brushSize = 2
        self.brushColor = Qt.black

		#Objects to tract the points
        self.lastPoint = QPoint()
        self.startLine = QPoint()
        self.endLine = QPoint()
        
        #Object to store line points
        self.listOfPoints = []
        
		#Menu Bar that stores options
        mainMenu = self.menuBar()

		#Creating menus for ults
        fileMenu = mainMenu.addMenu("File")
        drawingStages = mainMenu.addMenu("Drawing")
        b_size = mainMenu.addMenu("Brush Size")
      
        #Creating save action
        saveAction = QAction("Save", self)
        saveAction.setShortcut("Ctrl + S")
        fileMenu.addAction(saveAction)
        saveAction.triggered.connect(self.save)
        
		#Creating clear action
        clearAction = QAction("Clear", self)
        clearAction.setShortcut("Ctrl + C")
        fileMenu.addAction(clearAction)
        clearAction.triggered.connect(self.clear)
        

		#Creating Options for brush sizes; 4 pixle brush
        pix_4 = QAction("4px", self)
        b_size.addAction(pix_4)
        pix_4.triggered.connect(self.Pixel_4)

		#7 pixle brush
        pix_7 = QAction("7px", self)
        b_size.addAction(pix_7)
        pix_7.triggered.connect(self.Pixel_7)

        #9 pixle brush
        pix_9 = QAction("9px", self)
        b_size.addAction(pix_9)
        pix_9.triggered.connect(self.Pixel_9)

        #12 pixle brush
        pix_12 = QAction("12px", self)
        b_size.addAction(pix_12)
        pix_12.triggered.connect(self.Pixel_12)
        
        #FreeDraw
        freeDraw = QAction("Free Draw", self)
        drawingStages.addAction(freeDraw)
        freeDraw.triggered.connect(self.Free_Draw)
        
        #LineDraw
        LineDraw = QAction("Line Draw", self)
        drawingStages.addAction(LineDraw)
        LineDraw.triggered.connect(self.Line_Draw)
        
        #ContLineDraw
        ContLineDraw = QAction("Cont Line Draw", self)
        drawingStages.addAction(ContLineDraw)
        ContLineDraw.triggered.connect(self.Cont_Line_Draw)

    
	#Method for mouse clicks
    def mousePressEvent(self, event):

		#If left mouse button is pressed
        if event.button() == Qt.LeftButton:
            self.drawing = True
            self.lastPoint = event.pos()
        
        #Checks to see if startPoint needs to be at the end of the last drawn line
        if self.contLineDrawing:
            
            #Checks to see if there was a previous line drawn
            if self.endLine.isNull() or self.endLine.x() == -1:
                self.startLine = event.pos()
            else:
                self.startLine = self.endLine
        else:
            self.startLine = event.pos()
        print("Button Pressed")

	#Tracking the mouse when in free drawing mode
    def mouseMoveEvent(self, event):
		
		#Checking if left button is pressed and self drawing flag is true
        if (event.buttons() & Qt.LeftButton) & self.freeDrawing:
			
			#Creating painter object
            painter = QPainter(self.image)
            painter.setPen(QPen(self.brushColor, self.brushSize, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
			
			#Draw line from the last point of cursor to the current point
            painter.drawLine(self.lastPoint, event.pos())
            self.listOfPoints.append((self.lastPoint.x(), self.lastPoint.y()))
            print(self.listOfPoints)
			#Change the last point
            self.lastPoint = event.pos()
            self.update()
            

	#Method for mouse left button release
    def mouseReleaseEvent(self, event):

        if event.button() == Qt.LeftButton:
            self.drawing = False
            self.endLine = event.pos()
            print("Button Released")
            
            #Checking if left button is pressed and self drawing flag is true
            if self.lineDrawing or self.contLineDrawing:
    			
    			#Creating painter object
                painter = QPainter(self.image)
                painter.setPen(QPen(self.brushColor, self.brushSize, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
    			
    			#Draw line from the last point of cursor to the current point
                painter.drawLine(self.startLine, self.endLine) 
                
                if self.lineDrawing:  
                    self.listOfPoints.append((self.startLine.x(), self.startLine.y()))
                    self.listOfPoints.append((self.endLine.x(), self.startLine.y()))
                elif self.contLineDrawing:
                    self.listOfPoints.append((self.endLine.x(), self.endLine.y()))
                    
                print(self.listOfPoints)
                    
                self.update()

	#Paint event
    def paintEvent(self, event):
		
        # create a canvas
        canvasPainter = QPainter(self)
		
        # draw rectangle on the canvas
        canvasPainter.drawImage(self.rect(), self.image, self.image.rect())

	#Method for clearing every thing on canvas
    def clear(self):
        self.image.fill(Qt.white)
        self.listOfPoints = []
        self.endLine.setX(-1)
        self.update()

    def save(self):
        filePath, _ = QFileDialog.getSaveFileName(self, "Save Points", "",
                          "txt(*.txt);;All Files(*.*) ")
 
        if filePath == "":
            return
        
        with open(filePath, "w") as file:
            file.write('\n'.join([str(self.listOfPoints)]))
        
        
    
	#Methods for changing pixel sizes
    def Pixel_4(self):
        self.brushSize = 4

    def Pixel_7(self):
        self.brushSize = 7

    def Pixel_9(self):
        self.brushSize = 9

    def Pixel_12(self):
        self.brushSize = 12
        
    #Methods for Drawing Mode Flag System
    def Free_Draw(self):
        self.drawing = False
        self.freeDrawing = True
        self.lineDrawing = False
        self.contLineDrawing = False

    def Line_Draw(self):
        self.drawing = False
        self.freeDrawing = False
        self.lineDrawing = True
        self.contLineDrawing = False
    
    def Cont_Line_Draw(self):
        self.drawing = False
        self.freeDrawing = False
        self.lineDrawing = False
        self.contLineDrawing = True 
          
# create pyqt5 app
App = QApplication(sys.argv)

# create the instance of our Window
window = Window()

# showing the window
window.show()

# start the app
sys.exit(App.exec())
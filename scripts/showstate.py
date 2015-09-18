#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import *; 
from PyQt4.QtGui import *;
import sys



class MyWindow(QtGui.QWidget):
  path='';
  state=0;
  ipath='./examples/ET';
  def __init__(self):
    QtGui.QWidget.__init__(self)
    self.setWindowTitle('PyQt')       
    self.resize(600,80)        
    gridlayout = QtGui.QGridLayout()     
    self.labelInfo = QtGui.QLabel( "labelInfo" )
    self.labelInfo.setText( "Select a folder" )
    gridlayout.addWidget(self.labelInfo, 0,0,1,2)
    self.labelInfo1 = QtGui.QLabel( "labelInfo" )
    self.labelInfo1.setText( "Select a folder" )
    gridlayout.addWidget(self.labelInfo1, 1,0,1,2)
    self.labelInfo2 = QtGui.QLabel( "labelInfo" )
    self.labelInfo2.setText( "Select a folder" )
    gridlayout.addWidget(self.labelInfo2, 2,0,1,2)


    self.button1 = QtGui.QPushButton('Folder')   
    gridlayout.addWidget(self.button1, 3, 0)
    self.button2 = QtGui.QPushButton('Construct')   
    gridlayout.addWidget(self.button2, 3, 1)
    self.setLayout(gridlayout)       
    self.connect(self.button1,        
      QtCore.SIGNAL('clicked()'),     
      self.OnButton1)        
    self.connect(self.button2,        
      QtCore.SIGNAL('clicked()'),     
      self.OnButton2)    
    rospy.init_node('showstate', anonymous=True)
    self.workThread = WorkThread()
    self.connect( self.workThread, QtCore.SIGNAL("added(QStringList)"), self.add )
    self.workThread.start()
  def OnButton1(self):         
    self.labelInfo.setText(self.ipath)

  def OnButton2(self):         
    ipath=self.path

  def add(self,text):
    self.labelInfo.setText(text[0])
    self.labelInfo1.setText(text[1])
    self.labelInfo2.setText(text[2])


class WorkThread(QtCore.QThread):
  def __init__(self):
    QtCore.QThread.__init__(self)

  def __del__(self):
    self.wait()
  def callback(self,data):
    qs=QStringList();
    qs.append("Sine {0:.3f}".format(data.data[0]));
    qs.append("Time {0:.3f}".format(data.data[1]));
    qs.append("Diff {0:.3f}".format(data.data[2]));
    self.emit( QtCore.SIGNAL('added(QStringList)'), qs)

  def run(self):
    
    rospy.Subscriber("chatter", Float32MultiArray, self.callback)
    rospy.spin()
    

 
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    mywindow = MyWindow() 
    mywindow.show()
    app.exec_()
import sys
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from pylab import *
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

class Fenetre(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.setWindowTitle("Ma fenetre")
        self.layout = QVBoxLayout()

        self.fig = Figure()
        self.axes = self.fig.add_subplot(111)
 
        self.x = linspace(-pi, pi, 30)
        self.y = cos(self.x)
        self.line, = self.axes.plot(self.x, self.y)

        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)  # the matplotlib canvas

        self.bouton_cos = QPushButton("Cosinus")
        self.bouton_cos.clicked.connect(self.appui_cosinus)
        self.layout.addWidget(self.bouton_cos)

        self.bouton_sin = QPushButton("Sinus")
        self.bouton_sin.clicked.connect(self.appui_sinus)
        self.layout.addWidget(self.bouton_sin)

        self.setLayout(self.layout)
        self.show()

    def appui_cosinus(self):
        self.y = cos(self.x)
        self.line.set_ydata(self.y )
        self.canvas.draw()

    def appui_sinus(self):
        self.y = sin(self.x)
        self.line.set_ydata(self.y)
        self.canvas.draw()

app = QApplication.instance() 
if not app:
    app = QApplication(sys.argv)
fen = Fenetre()
app.exec_()
 

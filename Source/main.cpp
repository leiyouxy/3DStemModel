#include "TreePclQtGui.h"
#include <QtWidgets/QApplication>
#include <QMainWindow>

#include <vtkAutoInit.h>
#include <vtkRenderingOpenGLConfigure.h>

VTK_MODULE_INIT(vtkInteractionStyle);
//VTK_MODULE_INIT(vtkRenderingVolumeOpenGL)
VTK_MODULE_INIT(vtkRenderingFreeType)

#ifdef VTK_OPENGL2 
	VTK_MODULE_INIT(vtkRenderingOpenGL2)
#else 
	VTK_MODULE_INIT(vtkRenderingOpenGL)
#endif 

//VTK_MODULE_INIT(vtkRenderingFreeType)
//VTK_MODULE_INIT(vtkRenderingContextOpenGL)

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	TreePclQtGui w;	
	w.setWindowIcon(QIcon("tree.ico"));
	w.showMaximized();
	//w.showFullScreen();

	return a.exec();
}

#include "LoginPage.h"
#include <QtWidgets/QApplication>
#include <QDesktopWidget>

int main(int argc, char **argv)
{
	QApplication app(argc, argv);
	LoginPage w(argc, argv);
	w.show();
	w.move(app.desktop()->width()/2 - w.width()/2, app.desktop()->height()/2 - w.height()/2);
	return app.exec();
}

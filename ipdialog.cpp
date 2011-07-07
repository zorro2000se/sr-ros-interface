//Shadow Dexterous Hand - Windows ROS Interface Beta - AJPETHER 2011
//ipdialog.cpp
//ipdialog dialogbox containing textboxes to enter ROS master and local ip addresses.

#include "ipdialog.h"
#include "qtest.h"

IPDialog::IPDialog(QWidget *parent)
: QDialog(parent)
{
	setupUi(this);
	conn = 1;
}

IPDialog::~IPDialog()
{

}

//function takes contents of textboxes and passes them back to main application
void IPDialog::on_connectButton_clicked()
{
	QString Qmasteruri = masterEdit->text();
	masteruri = Qmasteruri.toStdString();
	QString Qlocalip = localEdit->text();
	localip = Qlocalip.toStdString();
}

void IPDialog::on_demoButton_clicked()
{
	conn = 0;
}

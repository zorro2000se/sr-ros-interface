//Shadow Dexterous Hand - Windows ROS Interface Beta - AJPETHER 2011
//ipdialog.h
//ipdialog dialogbox containing textboxes to enter ROS master and local ip addresses.
//header file initialises shared variables and functions

#ifndef IPDIALOG_H
#define IPDIALOG_H

#include <QDialog>
#include "ui_ipdialog.h"



class IPDialog : public QDialog, public Ui::IPDialog
{
	Q_OBJECT

public:
	IPDialog(QWidget *parent = 0);
	~IPDialog();
	std::string masteruri;
	std::string localip;
	bool conn;

	public slots:
		void on_connectButton_clicked();
		void on_demoButton_clicked();
	
	signals:
		void connectnow(int);
};

#endif // IPDIALOG_H

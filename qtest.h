//Shadow Dexterous Hand - Windows ROS Interface Beta - AJPETHER 2011
//qtest.h
//qtest application controls responses to user interaction with widgets in main window
//header file initialises shared variables and functions

#ifndef QTEST_H
#define QTEST_H

//#include <QtGui/QMainWindow>
#include "ui_qtest.h"
#include "ipdialog.h"
#include "grasping.h"

#include <QtGui>
//#include <QTest>

#include <string>
//#include <sstream>
//#include <iostream>
#include <QtDebug>
#include <QtXml/QDomDocument>
#include <QtXml/QDomElement>
#include <QFile>

#include "dllheader.h"

class Qtest : public QMainWindow
{
	Q_OBJECT

public:
	Qtest(QWidget *parent = 0, Qt::WFlags flags = 0);
	~Qtest();

	//xml variables and hash maps to store angles
	QDomDocument doc;
	QDomElement root;
	QHash<QString, float> jointhash;		
	QHash<QString, float> prevjointhash;	
	std::string joints[20];
	float angles[20];
	//std::string prevjoints[20];
	//float prevangles[20];
	int messages;
	QStringList grlist;
	QComboBox *lotsaboxes[20];
	int initialised;

	QString currtxt;
	QString nxttxt;
	float inter;
	float pause;

	int playrow;
	int playing;
	int numrows;
	QTimer playa;
	int loading;
	int stepsadded;
	int interpstep;
	int sent;
	int newstep;
	QHash<QString, bool> checkedhash;	
	QHash<QString, float> jointmaxhash;	
	QHash<QString, float> jointminhash;	
	QSignalMapper *addmappa;
	QSignalMapper *remmappa;
	//grasping *gsp;
	IPDialog *ipd;

	//public functions to respond to user interaction
	public slots:
		void on_setnowButton_clicked();
		void on_exitButton_clicked();
		void on_setnxtButton_clicked();
		int on_addlibButton_clicked();
		int loadxml();
		//void setgrasp();
		void on_comboBox_currentIndexChanged( const QString & text );
		void on_horizontalSlider_valueChanged(int val);
		void getangles();
		void setuptable();
		void on_saveButton_clicked();
		void on_playButton_clicked();
		void on_stopButton_clicked();
		void on_pauseButton_clicked();
		void on_loadButton_clicked();
		void addstep();
		void remstep();
		void on_verticalSlider_valueChanged(int val);
		void slidersChecked(QString slider, bool isit);
		void setrowadd(int thisrow);
		void setrowrem(int thisrow);
		void connectros(int mode);
		void showconnect();
		void on_actionGrasps_activated();
		void on_actionPlayer_activated();
		void on_actionSliders_activated();

		signals:
		void bing(QString, float);
		void slide(int);

private:
	Ui::QtestClass ui;

};

#endif // QTEST_H

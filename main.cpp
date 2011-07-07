//Shadow Dexterous Hand - Windows ROS Interface Beta - AJPETHER 2011
//Main.cpp Qt interface


#include "qtest.h"
#include "ipdialog.h"
//#include "releasecrash.h"
#include "rosslider.h"
#include <QtGui/QApplication>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkInterface>

int main(int argc, char *argv[])
{
	//qDebug()  << "**********************************************";
	//qDebug()  << "*             SHADOW ROBOT COMPANY           *";
	//qDebug()  << "*   WINDOWS ROS INTERFACE HAND DEMO SCRIPT   *";
	//qDebug()  << "*			    A PETHER - MARCH 2011          *";
	//qDebug()  << "**********************************************;";

	//initialise new application
	QApplication a(argc, argv);
	Qtest w;

	//initialise ROS network connection dialogue
	IPDialog dia(&w);
	//set textbox initial values then show dialog, execute application
	dia.masterEdit->setText("http://10.1.1.114:11311/");
	dia.localEdit->setText("10.1.1.142");
	dia.show();
	a.exec();

	//qDebug() << ;
	//qDebug() << "Setup Done";

	//Wired Connection
	//ros::master::setURI("http://10.1.2.24:11311/");
	//ros::network::setHost("10.1.2.25");

	//Wireless Connection

	//ros::master::setURI("http://192.168.1.108:11311/");
	//ros::master::setURI(dia.masteruri);
	//ros::network::setHost("192.168.1.102");
	//ros::network::setHost(dia.localip);
	//int argc1;

	//prepare command line arguaments to pass to ROSSetup in dll
	if(dia.conn == 1)
	{	
		char *argv1[1];

		QString blah = QCoreApplication::arguments().at(0);
		QByteArray   bytes  = blah.toAscii();
		char * ptr    = bytes.data();
		//qDebug() << ptr;
		int initialised = 0;

		//initialise connection to ROS server and wait for success
		////commented for gui demo
		initialised = ROSSetup(ptr, dia.masteruri, dia.localip);
		while(initialised == 0)
		{

		}
	}

	//add ROSSlider widgets to main window, set labels and positions accordingly
	//FIRST ROW OF SLIDERS

	ROSSlider WRJ1S(&w);
	WRJ1S.JointLabel->setText("WRJ1");
	WRJ1S.verticalSlider->setMinimum(-45);
	WRJ1S.verticalSlider->setMaximum(30);
	w.jointminhash["WRJ1"] = -45;
	w.jointmaxhash["WRJ1"] = 30;
	WRJ1S.move(-10,10);
	WRJ1S.show();

	ROSSlider WRJ2S(&w);
	WRJ2S.JointLabel->setText("WRJ2");
	WRJ2S.verticalSlider->setMinimum(-28);
	WRJ2S.verticalSlider->setMaximum(8);
	w.jointminhash["WRJ2"] = -28;
	w.jointmaxhash["WRJ2"] = 8;	
	WRJ2S.move(50,10);
	WRJ2S.show();

	ROSSlider THJ1S(&w);
	THJ1S.JointLabel->setText("THJ1");
	THJ1S.verticalSlider->setMinimum(0);
	THJ1S.verticalSlider->setMaximum(90);
	w.jointmaxhash["THJ1"] = 0;	
	w.jointminhash["THJ1"] = 90;
	THJ1S.move(110,10);
	THJ1S.show();

	ROSSlider THJ2S(&w);
	THJ2S.JointLabel->setText("THJ2");
	THJ2S.verticalSlider->setMinimum(-28);
	THJ2S.verticalSlider->setMaximum(28);
	w.jointminhash["THJ2"] = -28;
	w.jointmaxhash["THJ2"] = 28;
	THJ2S.move(170,10);
	THJ2S.show();

	ROSSlider THJ3S(&w);
	THJ3S.JointLabel->setText("THJ3");
	THJ3S.verticalSlider->setMinimum(-12);
	THJ3S.verticalSlider->setMaximum(12);
	w.jointminhash["THJ3"] = -12;
	w.jointmaxhash["THJ3"] = 12;
	THJ3S.move(230,10);
	THJ3S.show();

	ROSSlider THJ4S(&w);
	THJ4S.JointLabel->setText("THJ4");
	THJ4S.verticalSlider->setMinimum(0);
	THJ4S.verticalSlider->setMaximum(75);
	w.jointminhash["THJ4"] = 0;
	w.jointmaxhash["THJ4"] = 75;
	THJ4S.move(290,10);
	THJ4S.show();

	ROSSlider THJ5S(&w);
	THJ5S.JointLabel->setText("THJ5");
	THJ5S.verticalSlider->setMinimum(0);
	THJ5S.verticalSlider->setMaximum(75);
	w.jointminhash["THJ5"] = 0;
	w.jointmaxhash["THJ5"] = 75;
	THJ5S.move(350,10);
	THJ5S.show();

	ROSSlider FFJ0S(&w);
	FFJ0S.JointLabel->setText("FFJ0");
	FFJ0S.verticalSlider->setMinimum(0);
	FFJ0S.verticalSlider->setMaximum(180);
	w.jointminhash["FFJ0"] = 0;
	w.jointmaxhash["FFJ0"] = 180;
	FFJ0S.move(410,10);
	FFJ0S.show();

	ROSSlider FFJ3S(&w);
	FFJ3S.JointLabel->setText("FFJ3");
	FFJ3S.verticalSlider->setMinimum(0);
	FFJ3S.verticalSlider->setMaximum(90);
	w.jointminhash["FFJ3"] = 0;
	w.jointmaxhash["FFJ3"] = 90;
	FFJ3S.move(470,10);
	FFJ3S.show();

	ROSSlider FFJ4S(&w);
	FFJ4S.JointLabel->setText("FFJ4");
	FFJ4S.verticalSlider->setMinimum(-20);
	FFJ4S.verticalSlider->setMaximum(20);
	w.jointminhash["FFJ4"] = -20;
	w.jointmaxhash["FFJ4"] = 20;
	FFJ4S.move(530,10);
	FFJ4S.show();

	//SECOND ROW OF SLIDERS

	ROSSlider MFJ0S(&w);
	MFJ0S.JointLabel->setText("MFJ0");
	MFJ0S.verticalSlider->setMinimum(0);
	MFJ0S.verticalSlider->setMaximum(180);
	w.jointminhash["MFJ0"] = 0;
	w.jointmaxhash["MFJ0"] = 180;
	MFJ0S.move(-10,220);
	MFJ0S.show();

	ROSSlider MFJ3S(&w);
	MFJ3S.JointLabel->setText("MFJ3");
	MFJ3S.verticalSlider->setMinimum(0);
	MFJ3S.verticalSlider->setMaximum(90);
	w.jointminhash["MFJ3"] = 0;
	w.jointmaxhash["MFJ3"] = 90;
	MFJ3S.move(50,220);
	MFJ3S.show();

	ROSSlider MFJ4S(&w);
	MFJ4S.JointLabel->setText("MFJ4");
	MFJ4S.verticalSlider->setMinimum(-20);
	MFJ4S.verticalSlider->setMaximum(20);
	w.jointminhash["MFJ4"] = -20;
	w.jointmaxhash["MFJ4"] = 20;
	MFJ4S.move(110,220);
	FFJ4S.show();

	ROSSlider RFJ0S(&w);
	RFJ0S.JointLabel->setText("RFJ0");
	RFJ0S.verticalSlider->setMinimum(0);
	RFJ0S.verticalSlider->setMaximum(180);
	w.jointminhash["RFJ0"] = 0;
	w.jointmaxhash["RFJ0"] = 180;
	RFJ0S.move(170,220);
	RFJ0S.show();

	ROSSlider RFJ3S(&w);
	RFJ3S.JointLabel->setText("RFJ3");
	RFJ3S.verticalSlider->setMinimum(0);
	RFJ3S.verticalSlider->setMaximum(90);
	w.jointminhash["RFJ3"] = 0;
	w.jointmaxhash["RFJ3"] = 90;
	RFJ3S.move(230,220);
	RFJ3S.show();

	ROSSlider RFJ4S(&w);
	RFJ4S.JointLabel->setText("RFJ4");
	RFJ4S.verticalSlider->setMinimum(-20);
	RFJ4S.verticalSlider->setMaximum(20);
	w.jointminhash["RFJ4"] = -20;
	w.jointmaxhash["RFJ4"] = 20;
	RFJ4S.move(290,220);
	FFJ4S.show();

	ROSSlider LFJ0S(&w);
	LFJ0S.JointLabel->setText("LFJ0");
	LFJ0S.verticalSlider->setMinimum(0);
	LFJ0S.verticalSlider->setMaximum(180);
	w.jointminhash["LFJ0"] = 0;
	w.jointmaxhash["LFJ0"] = 180;
	LFJ0S.move(350,220);
	LFJ0S.show();

	ROSSlider LFJ3S(&w);
	LFJ3S.JointLabel->setText("LFJ3");
	LFJ3S.verticalSlider->setMinimum(0);
	LFJ3S.verticalSlider->setMaximum(90);
	w.jointminhash["LFJ3"] = 0;
	w.jointmaxhash["LFJ3"] = 90;
	LFJ3S.move(410,220);
	LFJ3S.show();

	ROSSlider LFJ4S(&w);
	LFJ4S.JointLabel->setText("LFJ4");
	LFJ4S.verticalSlider->setMinimum(-20);
	LFJ4S.verticalSlider->setMaximum(20);
	w.jointminhash["LFJ4"] = -20;
	w.jointmaxhash["LFJ4"] = 20;
	LFJ4S.move(470,220);
	LFJ4S.show();

	ROSSlider LFJ5S(&w);
	LFJ5S.JointLabel->setText("LFJ5");
	LFJ5S.verticalSlider->setMinimum(0);
	LFJ5S.verticalSlider->setMaximum(40);
	w.jointminhash["LFJ5"] = 0;
	w.jointmaxhash["LFJ5"] = 40;
	LFJ5S.move(530,220);
	LFJ5S.show();



	QObject::connect(&WRJ1S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&WRJ2S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));

	QObject::connect(&THJ1S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&THJ2S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&THJ3S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&THJ4S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&THJ5S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));

	QObject::connect(&FFJ0S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&FFJ3S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&FFJ4S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));

	QObject::connect(&MFJ0S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&MFJ3S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&MFJ4S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));

	QObject::connect(&RFJ0S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&RFJ3S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&RFJ4S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));

	QObject::connect(&LFJ0S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&LFJ3S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&LFJ4S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));
	QObject::connect(&LFJ5S, SIGNAL(boxChecked(QString, bool)),&w , SLOT(slidersChecked(QString, bool)));

	//LFJ5S.verticalSlider->setSliderPosition(LFJ5S.verticalSlider->minimum() + (LFJ5S.verticalSlider->maximum() / 3));

	//QObject::connect(&w, SIGNAL(slide(int)),&WRJ1S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&WRJ2S , SLOT(changeval(int)));

	//QObject::connect(&w, SIGNAL(slide(int)),&THJ1S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&THJ2S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&THJ3S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&THJ4S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&THJ5S , SLOT(changeval(int)));

	//QObject::connect(&w, SIGNAL(slide(int)),&FFJ0S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&FFJ3S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&FFJ4S , SLOT(changeval(int)));

	//QObject::connect(&w, SIGNAL(slide(int)),&MFJ0S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&MFJ3S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&MFJ4S , SLOT(changeval(int)));

	//QObject::connect(&w, SIGNAL(slide(int)),&RFJ0S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&RFJ3S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&RFJ4S , SLOT(changeval(int)));

	//QObject::connect(&w, SIGNAL(slide(int)),&LFJ0S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&LFJ3S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&LFJ4S , SLOT(changeval(int)));
	//QObject::connect(&w, SIGNAL(slide(int)),&LFJ5S , SLOT(changeval(int)));

	if(dia.conn == 1)
	{	
		QTimer *refresh = new QTimer(&w);
		//QTimer refresh;
		QObject::connect(&w, SIGNAL(bing(QString, float)),&WRJ1S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&WRJ2S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&THJ1S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&THJ2S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&THJ3S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&THJ4S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&THJ5S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&FFJ0S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&FFJ3S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&FFJ4S , SLOT(getpos(QString, float)));

		QObject::connect(&w, SIGNAL(bing(QString, float)),&MFJ0S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&MFJ3S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&MFJ4S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&RFJ0S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&RFJ3S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&RFJ4S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&LFJ0S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&LFJ3S , SLOT(getpos(QString, float)));
		QObject::connect(&w, SIGNAL(bing(QString, float)),&LFJ4S , SLOT(getpos(QString, float)));
		//QObject::connect(&w, SIGNAL(bing(QString, float)),&LFJ5S , SLOT(getpos(QString, float)));
		QObject::connect(refresh, SIGNAL(timeout()),&w , SLOT(getangles()));

		refresh->start(100);
	}

	//show the main window
	w.show();
	a.exec();
	return 0;
}

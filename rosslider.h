//Shadow Dexterous Hand - Windows ROS Interface Beta - AJPETHER 2011
//rosslider.h
//ROSSlider widget containing slider and labels for positional data
//header file initialises shared variables and functions

#ifndef ROSSLIDER_H
#define ROSSLIDER_H

#include <QWidget>
#include "ui_rosslider.h"
#include "qtest.h"

class ROSSlider : public QWidget, public Ui::ROSSlider
{
	Q_OBJECT

public:
	ROSSlider(QWidget *parent = 0);
	~ROSSlider();

	//unsigned short index_msg;
	//int msg_length;
	//float positions[28];
	QTimer *timer;

	public slots:
		void on_verticalSlider_valueChanged(int angle);
		void getpos(QString j, float pos);
		void on_checkBox_stateChanged();
		void changeval(int val);

signals:
		void boxChecked(QString, bool);

};

#endif // ROSSLIDER_H

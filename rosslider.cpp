//Shadow Dexterous Hand - Windows ROS Interface Beta - AJPETHER 2011
//rosslider.cpp
//ROSSlider widget containing slider and labels for positional data

#include "rosslider.h"

ROSSlider::ROSSlider(QWidget *parent)
: QWidget(parent)
{
	setupUi(this);
	//initialise timer to refresh current joint position
	timer = new QTimer(this);
	timer->setInterval(100);
	//connect(timer, SIGNAL(timeout()), this, SLOT(getpos()));
	//timer->start();
	//PosVal->setText("69");
	//QObject::connect(this, SIGNAL(boxChecked(int)), this, SLOT(valchanged(int)));
}

ROSSlider::~ROSSlider()
{

}

//sends new joint target to ROS dll when slider value changes
void ROSSlider::on_verticalSlider_valueChanged(int angle)
{

	//find which joint this slider represents
	QString Qjointname = JointLabel->text();
	std::string joint_name = Qjointname.toStdString();
	//ROSSend(joint_name, angle);

	//set up array of joints (this method should be changed)
	std::string joints[20] = {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"};
	float angles[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	//add the current joint at the start of the array
	joints[0] = joint_name;
	angles[0] = angle;

	//pass to ROS dll
	//std::string success;
	ROSSend(joints, angles);
}

void ROSSlider::getpos(QString j, float pos)
{
	//////find which joint this slider represents
	////QString Qjointname = JointLabel->text();
	////std::string joint_name = Qjointname.toStdString();
	////float receivepos = 0.07;//ROSRec(joint_name);				//POSITION UPDATE RECEIVE DISABLED
	//////convert to QString and update current position label
	////QString Qpos = QString::number(receivepos);
	////PosVal->setText(Qpos);
	if( j == JointLabel->text())
	{
		QString Qpos = QString::number(pos);
		PosVal->setText(Qpos);
	}
}

void ROSSlider::on_checkBox_stateChanged()
{
	//invoked when state of tick box changes
	//qDebug() << JointLabel->text() << "TICKBOX STATE CHANGED";
	emit boxChecked(JointLabel->text(),checkBox->isChecked());
}

void ROSSlider::changeval(int val)
{
	//if(checkBox->isChecked())
	//verticalSlider->setSliderPosition(val);
}
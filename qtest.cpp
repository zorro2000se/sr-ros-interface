#include "qtest.h"
#include "ipdialog.h"
#include "rosslider.h"

Qtest::Qtest(QWidget *parent, Qt::WFlags flags)
: QMainWindow(parent, flags)
{
	ui.setupUi(this);

	loading = 0;
	//numrows = 0;
	loadxml();
	setuptable();
	addstep();

	sent = 0;
	playrow = 0;
	playing = 0;
	numrows = 0;
	initialised = 1;
	playa.setSingleShot(true);
	connect(&playa, SIGNAL(timeout()),this, SLOT(on_playButton_clicked()));

	QString qte;
	std::string qt;
	qte, qt = 7;

	QMessageBox *aboutBox = new QMessageBox(this);
	aboutBox->setText("Shadow Robot Company 2011 - andy@shadowrobot.com");
	aboutBox->setWindowTitle("About This Software");
	connect(ui.actionAbout_This_Software, SIGNAL(activated()), aboutBox, SLOT(exec()));

	ui.actionBiomorphic_Arm->setCheckable(1);
	ui.action5_Finger->setCheckable(1);
	ui.action3_Finger->setCheckable(1);
	ui.action1_Finger->setCheckable(1);

	QActionGroup *handgroup = new QActionGroup(this);
	handgroup->addAction(ui.action5_Finger);
	handgroup->addAction(ui.action3_Finger);
	handgroup->addAction(ui.action1_Finger);
	handgroup->addAction(ui.actionCustom);

	ui.action5_Finger->trigger();

	connect(ui.actionConnect, SIGNAL(activated()), this, SLOT(showconnect()));
	showconnect();

	ui.frame->hide();
	ui.frame_2->hide();

	//ui.playerwindow->setAttribute(Qt::WA_DeleteOnClose);
	//ui.mdiArea->removeSubWindow(ui.playerwindow);

	//ui.playerwindow->setWindowTitle("Player");
	////ui.playerwindow->setWindowSize("380, 200");
	//ui.graspswindow->setWindowTitle("Grasps");
	//QMdiSubWindow *subw = new QMdiSubWindow;
	//subw->setWindowTitle("Sliders");


	//	//sliders could be added like this...
	//ROSSlider *WRJ1Sv = new ROSSlider(subw);
	//WRJ1Sv->JointLabel->setText("TESTING");
	//WRJ1Sv->verticalSlider->setMinimum(-45);
	//WRJ1Sv->verticalSlider->setMaximum(30);
	//jointminhash["WRJ1"] = -45;
	//jointmaxhash["WRJ1"] = 30;
	//WRJ1Sv->move(-10,30);
	////WRJ1Sv->show();

	//graspswin *graspsw = new graspswin(subw);
	//QMdiSubWindow *subw = new QMdiSubWindow;
	//subw->setWidget(WRJ1Sv);
	//mdiArea.addSubWindow(subw);
	//ui.mdiArea->addSubWindow(subw);


}

Qtest::~Qtest()
{
}

//loads grasps from xml file
int Qtest::loadxml()
{
	//open specified xml file and load into dom document
	//QDomDocument doc;
	QFile file( "grasps.xml" );
	if( !file.open(QIODevice::ReadOnly) )
	{
		qDebug() << "not read only";
		return -1;
	}

	if( !doc.setContent( &file ) )
	{
		qDebug() << "setcontent fail";
		file.close();
		return -2;
	}
	file.close();

	//extract the grasp names from the document and add them to the grasp select combobox
	root = doc.documentElement();
	if( root.tagName() != "root" )
	{
		qDebug() << "incorrect xml file";
		return -3;
	}

	ui.comboBox->clear();
	QDomNode n = root.firstChild();
	while( !n.isNull() )
	{
		QDomElement e = n.toElement();
		if( !e.isNull() )
		{
			if( e.tagName() == "grasp" )
			{
				ui.comboBox->addItem(e.attribute("name"));
				grlist << e.attribute("name");
			}
		}
		n = n.nextSibling();				//NEXT STEP - LOOP THROUGH ALL GRASPS
	}
	return 0;

}

//when a grasp is selected from the combobox, extract all of the corresponding joint angles from the dom document
void Qtest::on_comboBox_currentIndexChanged(const QString & text)
{
	//QHash<QString, float> jointhash;
	QDomNode n = root.firstChild();
	while( !n.isNull() )
	{
		QDomElement e = n.toElement();
		if( !e.isNull() )
		{
			if(e.attribute("name") == text)
			{
				qDebug() << e.attribute("name");
				QDomNode m = n.firstChild();
				QDomElement f = m.toElement();

				for(int i = 0; i < 20; i++)
				{
					if( !f.isNull() )
					{
						joints[i] = f.attribute("name").toStdString();
						angles[i] = f.text().toFloat();
						jointhash[f.attribute("name")] = f.text().toFloat();
						//qDebug() << f.attribute("name") << f.text();
						m = m.nextSibling();
						f = m.toElement();
						//i++;
					}
					else
					{
						joints[i] = "0";
					}
				}
			}
		}
		n = n.nextSibling();				//NEXT STEP - LOOP THROUGH ALL GRASPS

	}


}
//passes the joint angles of the current grasp to the ros dll
void Qtest::on_setnowButton_clicked()
{
	//qDebug() << "GRASP SET" << ui.comboBox->currentText();@
	ROSSend(joints, angles);
}
//on click of exit button, closes ros connection then quits application
void Qtest::on_exitButton_clicked()
{
	ROSQuit();
	QCoreApplication::quit();
}

//stores and displays second joint for slider to iterate too/from. NEEDS EDITING
void Qtest::on_setnxtButton_clicked()
{
	prevjointhash = jointhash;
	qDebug() << "NEXT GRASP SET" << ui.comboBox->currentText();
	ui.label_2->setText(ui.comboBox->currentText());
}

//interpolates between two grasps, based on the value of a horizontal slider
void Qtest::on_horizontalSlider_valueChanged(int val)
{
	QString joint[20] = {"WRJ1","WRJ2","THJ1","THJ2","THJ3","THJ4","THJ5","FFJ0","FFJ3","FFJ4","MFJ0","MFJ3","MFJ4","RFJ0","RFJ3","RFJ4","LFJ0","LFJ3","LFJ4","LFJ5"};
	std::string gotback;
	for(int j = 0; j < 20; j++)
	{
		joints[j] = joint[j].toStdString();
		angles[j] = prevjointhash[joint[j]] - ((prevjointhash[joint[j]] - jointhash[joint[j]]) / 100 * val);
	}
	//qDebug() << val;	
	on_setnowButton_clicked();
}

//adds the current grasp configuration to the xml grasp file with the name currently contained in the lineedit textbox
int Qtest::on_addlibButton_clicked()
{
	std::string joint[20] = {"WRJ1","WRJ2","THJ1","THJ2","THJ3","THJ4","THJ5","FFJ0","FFJ3","FFJ4","MFJ0","MFJ3","MFJ4","RFJ0","RFJ3","RFJ4","LFJ0","LFJ3","LFJ4","LFJ5"};

	//QDomNode n = root.firstChild();
	QDomElement ng = doc.createElement("grasp");
	ng.setAttribute("name", ui.lineEdit->text());
	for(int j = 0; j < 20; j++)
	{
		QDomElement nj = doc.createElement("joint");
		nj.setAttribute("name", joint[j].c_str());
		//Qstring curpos = ROSRec(joint[j].c_str());

		//QDomText text = doc.createTextNode(QString::number(ROSRec(joint[j]))); //PUT BACK IN TO SAVE CORRECT ANGLES
		QDomText text = doc.createTextNode("999");
		nj.appendChild(text);
		ng.appendChild(nj);
	}

	root.appendChild(ng);

	QFile file( "grasps.xml" );
	if( !file.open(QIODevice::WriteOnly) )
		return -1;

	QTextStream ts( &file );
	ts << doc.toString();

	file.close();
	loadxml();
	return 0;
}

//gets joint angles from ROS communications library and transmits them to slider displays
void Qtest::getangles()
{
	std::string jointlist[20] = {"WRJ1","WRJ2","THJ1","THJ2","THJ3","THJ4","THJ5","FFJ0","FFJ3","FFJ4","MFJ0","MFJ3","MFJ4","RFJ0","RFJ3","RFJ4","LFJ0","LFJ3","LFJ4","LFJ5"};

		//float pos = ROSRec("FFJ0");
		//emit bing("FFJ0", pos);

	for(int i = 0; i < 20; i++)
	{

		float pos = ROSRec(jointlist[i]);
		//float pos = 6.6;
		emit bing(jointlist[i].c_str(), pos);
		qDebug() << jointlist[i].c_str() << pos;
	}
}

//sets up the dimensions of the table which will contain player scripts
void Qtest::setuptable()
{
	//QTableWidgetItem *newItem = new QTableWidgetItem("here");
	//ui.tableWidget->setItem(1, 1, newItem);
	ui.tableWidget->setColumnWidth(0,120);
	ui.tableWidget->setColumnWidth(1,40);
	ui.tableWidget->setColumnWidth(2,40);
	ui.tableWidget->setColumnWidth(3,20);
	ui.tableWidget->setColumnWidth(4,20);
	ui.tableWidget->setRowHeight(0,20);
}

//executed when player play button clicked, will move the hand to the next grasp, either directly or interpolated
void Qtest::on_playButton_clicked()
{	
	//check the current play state
	if(playing == 3)
	{
		playing = 1;
		return;
	}
	if(playing == 2)
	{
		playing = 0;
		return;
	}
	if(playing == 0)
	{
		playrow = 0;
		playing = 1;
		newstep = 1;
		//interpstep = 0;
	}

	//if moved onto a new row, get all of the appropriate information from the table
	if(newstep == 1)
	{
		interpstep = 0;
		newstep = 0;
		numrows = ui.tableWidget->rowCount();// - 1;				
		ui.tableWidget->setCurrentCell(playrow, 0);
		QComboBox* combo;

		//combo=(QComboBox*)ui.tableWidget->cellWidget(0, 0);
		if(playrow == 0)combo=(QComboBox*)ui.tableWidget->cellWidget(numrows-1, 0);
		else combo=(QComboBox*)ui.tableWidget->cellWidget(playrow-1, 0);

		//combo=(QComboBox*)ui.tableWidget->cellWidget(playrow+1, 0);
		nxttxt = combo->currentText();
		on_comboBox_currentIndexChanged(nxttxt);
		prevjointhash = jointhash;

		combo=(QComboBox*)ui.tableWidget->cellWidget(playrow, 0);
		currtxt = combo->currentText();
		on_comboBox_currentIndexChanged(currtxt);

		QLineEdit* edit=(QLineEdit*)ui.tableWidget->cellWidget(playrow, 1);
		pause = edit->text().toFloat();

		edit=(QLineEdit*)ui.tableWidget->cellWidget(playrow, 2);
		inter = edit->text().toFloat();

	}
	//take next interpolation step if required
	if(inter != 0)
	{

		QString joint[20] = {"WRJ1","WRJ2","THJ1","THJ2","THJ3","THJ4","THJ5","FFJ0","FFJ3","FFJ4","MFJ0","MFJ3","MFJ4","RFJ0","RFJ3","RFJ4","LFJ0","LFJ3","LFJ4","LFJ5"};

		for(int j = 0; j < 20; j++)
		{
			joints[j] = joint[j].toStdString();
			angles[j] = prevjointhash[joint[j]] - ((prevjointhash[joint[j]] - jointhash[joint[j]]) / 10 * interpstep);
			//angles[j] = jointhash[joint[j]] - (jointhash[joint[j]] - (prevjointhash[joint[j]] / 10 * interpstep));
			////angles[j] = jointhash[joint[j]] - (prevjointhash[joint[j]] / 10 * interpstep);

		}

		on_setnowButton_clicked();

		if (interpstep > 9)
		{
			playrow++;
			newstep = 1;
			if(playrow == numrows)playrow = 0;
			playa.start(pause*1000);
		}
		else
		{
			playa.start(inter*100);
			interpstep++;
		}

	}
	else
	{
		//if not interpolating, do this
		on_setnowButton_clicked();
		playrow++;
		newstep = 1;

		if(playrow == numrows)
			playrow = 0;
		//if(playing != 2)
		playa.start(pause*1000);

	}
}

//when the player stop button is clicked,update play state variable
void Qtest::on_stopButton_clicked()
{
	playing = 2;
	getangles();
}

//when the player pause button is clicked,update play state variable
void Qtest::on_pauseButton_clicked()
{
	if(playing == 3)
	{
		playing = 1;
		ui.pauseButton->setText("Pause");
		playa.start(10);
	}
	else
	{
		playing = 3;
		ui.pauseButton->setText("Resume");
	}
}

//when player load button is clicked, open file browser and load contents of file into table
void Qtest::on_loadButton_clicked()
{

	//allow user to select file
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open Player File"), QDir::currentPath(), tr("Extensionless XML Files (*.*)"));

	qDebug() << fileName;

	QFile file( fileName );
	QDomDocument loaddoc;

	QMessageBox msgBox;
	msgBox.setText("The selected file is not a valid movement player file.");
	msgBox.setWindowTitle("File Load Error");

	//attempt to load contents of selected xml file
	if( !file.open(QIODevice::ReadOnly) )
	{
		qDebug() << "failed to open";
		msgBox.exec();
		return ;
	}

	if( !loaddoc.setContent( &file ) )
	{
		qDebug() << "setcontent fail";
		msgBox.exec();
		file.close();
		return;
	}
	file.close();


	QDomElement loadroot = loaddoc.documentElement();

	if( loadroot.tagName() != "movement" )
	{
		qDebug() << "incorrect xml file";
		msgBox.exec();
		return;
	}

	//extract the grasp names from the document and add them to the grasp select combobox
	ui.tableWidget->clear();
	stepsadded = 0;
	loading = 1;
	QDomNode n = loadroot.firstChild();
	while( !n.isNull() )
	{
		QDomElement e = n.toElement();
		if( !e.isNull() )
		{
			if( e.tagName() == "step" )
			{
				QDomNode m = n.firstChild();
				QDomElement f = m.toElement();
				//ui.tableWidget->setCurrentCell(stepsadded,0);
				addstep();
				QComboBox* combo=(QComboBox*)ui.tableWidget->cellWidget(stepsadded, 0);
				QLineEdit* pause=(QLineEdit*)ui.tableWidget->cellWidget(stepsadded, 1);
				QLineEdit* inter=(QLineEdit*)ui.tableWidget->cellWidget(stepsadded, 2);
				for(int i = 0; i < grlist.length(); i++)
				{
					if(grlist[i] == f.attribute("name"))
					{
						combo->setCurrentIndex(i);
						qDebug() << i << "/" << grlist.length() << f.attribute("name") << combo->itemText(i);
						m = m.nextSibling();
						f = m.toElement();
						pause->setText(f.text());
						m = m.nextSibling();
						f = m.toElement();
						inter->setText(f.text());

						break;
					}
				}
				//qDebug() << i << "/" << grlist.length() << f.attribute("name");
				//combo = 0;
			}
		}
		n = n.nextSibling();				//NEXT STEP - LOOP THROUGH ALL GRASPS
		stepsadded++;
		ui.tableWidget->removeRow(stepsadded);
	}
	//return 0;
	loading = 0;
}

//when player save button is clicked, open file browser to select save file then then save contents of table into this file
void Qtest::on_saveButton_clicked()
{
	numrows = ui.tableWidget->rowCount();
	//on_playButton_clicked();
	//on_stopButton_clicked();
	QDomDocument playerdoc;
	//QDomNode n = root.firstChild();
	QDomElement nm = playerdoc.createElement("movement");

	for(int thisrow = 0; thisrow < numrows; thisrow++)
	{
		QDomElement ns = playerdoc.createElement("step");
		QDomElement ng = playerdoc.createElement("grasp");
		QComboBox* combo=(QComboBox*)ui.tableWidget->cellWidget(thisrow, 0);
		ng.setAttribute("name", combo->currentText());
		ns.appendChild(ng);

		QLineEdit* edit=(QLineEdit*)ui.tableWidget->cellWidget(thisrow, 1);
		QDomElement np = playerdoc.createElement("pause_time");
		QDomText npt = doc.createTextNode(edit->text());
		np.appendChild(npt);
		ns.appendChild(np);

		edit=(QLineEdit*)ui.tableWidget->cellWidget(thisrow, 2);
		QDomElement ni = playerdoc.createElement("interpolation_time");
		QDomText nit = doc.createTextNode(edit->text());
		ni.appendChild(nit);
		ns.appendChild(ni);

		QDomElement nl = playerdoc.createElement("loop_to_step");
		QDomText nlt = doc.createTextNode("0.0");
		nl.appendChild(nlt);
		ns.appendChild(nl);

		QDomElement nn = playerdoc.createElement("number_loops");
		QDomText nnt = doc.createTextNode("0.0");
		nn.appendChild(nnt);
		ns.appendChild(nn);

		nm.appendChild(ns);
		qDebug() << ns.tagName() << ns.text();

	}


	playerdoc.appendChild(nm);

	QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save Player File"), QDir::currentPath(), tr("Extensionless XML Files (*.*)"));

	QFile file( fileName );

	if( !file.open(QIODevice::WriteOnly) )
	{
		qDebug() << "player xml write failed";
		return;
	}

	QTextStream ts( &file );
	ts << playerdoc.toString();

	file.close();
	qDebug() << "player xml write complete";
	//loadxml();
}

//add a new row to the player table, with the appropriate widgets in each cell
void Qtest::addstep()
{
	int row = ui.tableWidget->currentRow();
	row++;
	if(row == -1)row = 0;
	if(loading == 1)row = stepsadded;
	qDebug() << row;

	ui.tableWidget->insertRow(row);
	ui.tableWidget->setRowHeight(row,20);

	lotsaboxes[row] = new QComboBox();
	lotsaboxes[row]->addItems(grlist);
	ui.tableWidget->setCellWidget(row,0,lotsaboxes[row]);
	QPushButton *addstepButton= new QPushButton();
	QPushButton *remstepButton = new QPushButton();

	QLineEdit* pause = new QLineEdit();
	pause->setText("2.0");
	ui.tableWidget->setCellWidget(row,1,pause);

	QLineEdit* inter = new QLineEdit();
	inter->setText("0.0");
	ui.tableWidget->setCellWidget(row,2,inter);

	//QPushButton *addstepButton_1 = new QPushButton();
	addstepButton->setText("+");
	ui.tableWidget->setCellWidget(row,3,addstepButton);

	addmappa = new QSignalMapper(this);
	connect(addstepButton, SIGNAL(clicked()), addmappa, SLOT(map()));	
	//connect(addstepButton, SIGNAL(clicked()), this, SLOT(addstep()));	
	addmappa->setMapping(addstepButton, row);
	//qDebug() << "blip" << row;
	connect(addmappa, SIGNAL(mapped(int)), this, SLOT(setrowadd(int)));

	//QPushButton *remstepButton_1 = new QPushButton();
	remstepButton->setText("-");
	ui.tableWidget->setCellWidget(row,4,remstepButton);
	
	remmappa = new QSignalMapper(this);
	connect(remstepButton, SIGNAL(clicked()), remmappa, SLOT(map()));	
	//connect(addstepButton, SIGNAL(clicked()), this, SLOT(addstep()));	
	remmappa->setMapping(remstepButton, row);
	//qDebug() << "blip" << row;
	connect(remmappa, SIGNAL(mapped(int)), this, SLOT(setrowrem(int)));	

}

//remove the appropriate row from the player table
void Qtest::remstep()
{
	if(ui.tableWidget->rowCount() > 1)
	{
		int row = ui.tableWidget->currentRow();
		qDebug() << row;
		ui.tableWidget->removeRow(row);
	}
}

//executed when master slider is moved, will move all joints that have checkbox selected
void Qtest::on_verticalSlider_valueChanged(int val)
{

	std::string jointlist[20] = {"WRJ1","WRJ2","THJ1","THJ2","THJ3","THJ4","THJ5","FFJ0","FFJ3","FFJ4","MFJ0","MFJ3","MFJ4","RFJ0","RFJ3","RFJ4","LFJ0","LFJ3","LFJ4","LFJ5"};
	QString QJointlist[20] = {"WRJ1","WRJ2","THJ1","THJ2","THJ3","THJ4","THJ5","FFJ0","FFJ3","FFJ4","MFJ0","MFJ3","MFJ4","RFJ0","RFJ3","RFJ4","LFJ0","LFJ3","LFJ4","LFJ5"};

	int index = 0;


	for(int i = 0; i < 20; i++)
	{
		if (checkedhash[QJointlist[i]] == 1)
		{
			//QString jlist = QString::fromStdString(jointlist[i]);
			//QString jlist = QJointlist[i];
			joints[index] = jointlist[i];
			angles[index] = (jointminhash[QJointlist[i]] + (jointmaxhash[QJointlist[i]] / 100 * val));
			qDebug() << QJointlist[i] << angles[index];
			index++;
		}
	}
	joints[index] = "0";
	on_setnowButton_clicked();
}

//updates the hash list containing checkbox values when the state of one is changed
void Qtest::slidersChecked(QString slider, bool isit)
{
	checkedhash[slider] = isit;
	//std::string stdsli = slider.toStdString();
	//checkedhash[stdsli] = 0;
	//qDebug() << "sliderchecked" << slider << isit;
}

//sets focus on correct row of table for adding a row
void Qtest::setrowadd(int thisrow)
{
	//qDebug() << "Bop" << thisrow;
	ui.tableWidget->setCurrentCell(thisrow, 0);
	addstep();
}

//sets focus on correct row of table for removing a row
void Qtest::setrowrem(int thisrow)
{
	//qDebug() << "Bop" << thisrow;
	ui.tableWidget->setCurrentCell(thisrow, 0);
	remstep();
}

void Qtest::connectros(int mode)
{
	if(mode == 1)
	{
		ui.statusBar->showMessage("Connecting", 0);
		char *argv1[1];

		QString blah = QCoreApplication::arguments().at(0);
		QByteArray   bytes  = blah.toAscii();
		char * ptr    = bytes.data();
		//qDebug() << ptr;
		int initialised = 0;

		//initialise connection to ROS server and wait for success
		////commented for gui demo
		initialised = ROSSetup(ptr, ipd->masteruri, ipd->localip);

		//while(initialised == 0)
		//{

		//}
		ui.statusBar->clearMessage();
		QLabel *lbl = new QLabel(this);
		if (initialised != 0)
		{
			lbl->setText("Connection Established");
			ui.statusBar->addPermanentWidget(lbl);
		}
		else
		{
			lbl->setText("Connection Failed");
			ui.statusBar->addPermanentWidget(lbl);
		}
	}
	else
	{
		QLabel *lbl = new QLabel(this);
		lbl->setText("GUI Demo Mode");
		ui.statusBar->addPermanentWidget(lbl);
		//ui.statusBar->showMessage("GUI Demo Mode", 0);
	}
}


void Qtest::showconnect()
{
	ipd = new IPDialog(this);
	connect(ipd, SIGNAL(connectnow(int)), this, SLOT(connectros(int)));
	ipd->masterEdit->setText("http://10.1.2.23:11311/");
	ipd->localEdit->setText("10.1.2.24");
	ipd->show();
}

void Qtest::on_actionGrasps_activated()
{
	QMdiSubWindow *subg = new QMdiSubWindow;
	subg->setWidget(ui.frame);
	subg->setMinimumSize(345, 185);
	ui.mdiArea->addSubWindow(subg);
	subg->show();
	ui.frame->show();
}

void Qtest::on_actionPlayer_activated()
{
	QMdiSubWindow *subp = new QMdiSubWindow;
	subp->setWidget(ui.frame_2);
	subp->setMinimumSize(395, 220);
	ui.mdiArea->addSubWindow(subp);
	subp->show();
	ui.frame_2->show();
}


void Qtest::on_actionSliders_activated()
{

	QMdiSubWindow *subs = new QMdiSubWindow;
	//subs->setWidget(ui.frame_2);
	subs->setMinimumSize(620, 480);

	//QFrame *subs = new QFrame(subw);
	//subs->setMinimumSize(395, 220);

	////////add ROSSlider widgets to main window, set labels and positions accordingly
	////////FIRST ROW OF SLIDERS

	int ypos = 30;

	ROSSlider *WRJ1S = new ROSSlider(subs);
	WRJ1S->JointLabel->setText("WRJ1");
	WRJ1S->verticalSlider->setMinimum(-45);
	WRJ1S->verticalSlider->setMaximum(30);
	jointminhash["WRJ1"] = -45;
	jointmaxhash["WRJ1"] = 30;
	WRJ1S->move(-10,ypos);

	ROSSlider *WRJ2S = new ROSSlider(subs);
	WRJ2S->JointLabel->setText("WRJ2");
	WRJ2S->verticalSlider->setMinimum(-28);
	WRJ2S->verticalSlider->setMaximum(8);
	jointminhash["WRJ2"] = -28;
	jointmaxhash["WRJ2"] = 8;
	WRJ2S->move(50,ypos);

	ROSSlider *THJ1S = new ROSSlider(subs);
	THJ1S->JointLabel->setText("THJ1");
	THJ1S->verticalSlider->setMinimum(0);
	THJ1S->verticalSlider->setMaximum(90);
	jointmaxhash["THJ1"] = 0;	
	jointminhash["THJ1"] = 90;
	THJ1S->move(110,ypos);

	ROSSlider *THJ2S = new ROSSlider(subs);
	THJ2S->JointLabel->setText("THJ2");
	THJ2S->verticalSlider->setMinimum(-28);
	THJ2S->verticalSlider->setMaximum(28);
	jointminhash["THJ2"] = -28;
	jointmaxhash["THJ2"] = 28;
	THJ2S->move(170,ypos);

	ROSSlider *THJ3S = new ROSSlider(subs);
	THJ3S->JointLabel->setText("THJ3");
	THJ3S->verticalSlider->setMinimum(-12);
	THJ3S->verticalSlider->setMaximum(12);
	jointminhash["THJ3"] = -12;
	jointmaxhash["THJ3"] = 12;
	THJ3S->move(230,ypos);

	ROSSlider *THJ4S = new ROSSlider(subs);
	THJ4S->JointLabel->setText("THJ4");
	THJ4S->verticalSlider->setMinimum(0);
	THJ4S->verticalSlider->setMaximum(75);
	jointminhash["THJ4"] = 0;
	jointmaxhash["THJ4"] = 75;
	THJ4S->move(290,ypos);

	ROSSlider *THJ5S = new ROSSlider(subs);
	THJ5S->JointLabel->setText("THJ5");
	THJ5S->verticalSlider->setMinimum(-60);
	THJ5S->verticalSlider->setMaximum(60);
	jointminhash["THJ5"] = -60;
	jointmaxhash["THJ5"] = 60;
	THJ5S->move(350,ypos);

	ROSSlider *FFJ0S = new ROSSlider(subs);
	FFJ0S->JointLabel->setText("FFJ0");
	FFJ0S->verticalSlider->setMinimum(0);
	FFJ0S->verticalSlider->setMaximum(180);
	jointminhash["FFJ0"] = 0;
	jointmaxhash["FFJ0"] = 180;
	FFJ0S->move(410,ypos);

	ROSSlider *FFJ3S = new ROSSlider(subs);
	FFJ3S->JointLabel->setText("FFJ3");
	FFJ3S->verticalSlider->setMinimum(0);
	FFJ3S->verticalSlider->setMaximum(90);
	jointminhash["FFJ3"] = 0;
	jointmaxhash["FFJ3"] = 90;
	FFJ3S->move(470,ypos);

	ROSSlider *FFJ4S = new ROSSlider(subs);
	FFJ4S->JointLabel->setText("FFJ4");
	FFJ4S->verticalSlider->setMinimum(-20);
	FFJ4S->verticalSlider->setMaximum(20);
	jointminhash["FFJ4"] = -20;
	jointmaxhash["FFJ4"] = 20;
	FFJ4S->move(530,ypos);

	//SECOND ROW OF SLIDERS
	ypos = 240;

	ROSSlider *MFJ0S = new ROSSlider(subs);
	MFJ0S->JointLabel->setText("MFJ0");
	MFJ0S->verticalSlider->setMinimum(0);
	MFJ0S->verticalSlider->setMaximum(180);
	jointminhash["MFJ0"] = 0;
	jointmaxhash["MFJ0"] = 180;
	MFJ0S->move(-10,ypos);

	ROSSlider *MFJ3S = new ROSSlider(subs);
	MFJ3S->JointLabel->setText("MFJ3");
	MFJ3S->verticalSlider->setMinimum(0);
	MFJ3S->verticalSlider->setMaximum(90);
	jointminhash["MFJ3"] = 0;
	jointmaxhash["MFJ3"] = 90;
	MFJ3S->move(50,ypos);

	ROSSlider *MFJ4S = new ROSSlider(subs);
	MFJ4S->JointLabel->setText("MFJ4");
	MFJ4S->verticalSlider->setMinimum(-20);
	MFJ4S->verticalSlider->setMaximum(20);
	jointminhash["MFJ4"] = -20;
	jointmaxhash["MFJ4"] = 20;
	MFJ4S->move(110,ypos);

	ROSSlider *RFJ0S = new ROSSlider(subs);
	RFJ0S->JointLabel->setText("RFJ0");
	RFJ0S->verticalSlider->setMinimum(0);
	RFJ0S->verticalSlider->setMaximum(180);
	jointminhash["RFJ0"] = 0;
	jointmaxhash["RFJ0"] = 180;
	RFJ0S->move(170,ypos);

	ROSSlider *RFJ3S = new ROSSlider(subs);
	RFJ3S->JointLabel->setText("RFJ3");
	RFJ3S->verticalSlider->setMinimum(0);
	RFJ3S->verticalSlider->setMaximum(90);
	jointminhash["RFJ3"] = 0;
	jointmaxhash["RFJ3"] = 90;
	RFJ3S->move(230,ypos);

	ROSSlider *RFJ4S = new ROSSlider(subs);
	RFJ4S->JointLabel->setText("RFJ4");
	RFJ4S->verticalSlider->setMinimum(-20);
	RFJ4S->verticalSlider->setMaximum(20);
	jointminhash["RFJ4"] = -20;
	jointmaxhash["RFJ4"] = 20;
	RFJ4S->move(290,ypos);

	ROSSlider *LFJ0S = new ROSSlider(subs);
	LFJ0S->JointLabel->setText("LFJ0");
	LFJ0S->verticalSlider->setMinimum(0);
	LFJ0S->verticalSlider->setMaximum(180);
	jointminhash["LFJ0"] = 0;
	jointmaxhash["LFJ0"] = 180;
	LFJ0S->move(350,ypos);

	ROSSlider *LFJ3S = new ROSSlider(subs);
	LFJ3S->JointLabel->setText("LFJ3");
	LFJ3S->verticalSlider->setMinimum(0);
	LFJ3S->verticalSlider->setMaximum(90);
	jointminhash["LFJ3"] = 0;
	jointmaxhash["LFJ3"] = 90;
	LFJ3S->move(410,ypos);

	ROSSlider *LFJ4S = new ROSSlider(subs);
	LFJ4S->JointLabel->setText("LFJ4");
	LFJ4S->verticalSlider->setMinimum(-20);
	LFJ4S->verticalSlider->setMaximum(20);
	jointminhash["LFJ4"] = -20;
	jointmaxhash["LFJ4"] = 20;
	LFJ4S->move(470,ypos);

	ROSSlider *LFJ5S = new ROSSlider(subs);
	LFJ5S->JointLabel->setText("LFJ5");
	LFJ5S->verticalSlider->setMinimum(0);
	LFJ5S->verticalSlider->setMaximum(40);
	jointminhash["LFJ5"] = 0;
	jointmaxhash["LFJ5"] = 40;
	LFJ5S->move(530,ypos);

	QObject::connect(WRJ1S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(WRJ2S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));

	QObject::connect(THJ1S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(THJ2S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(THJ3S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(THJ4S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(THJ5S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));

	QObject::connect(FFJ0S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(FFJ3S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(FFJ4S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));

	QObject::connect(MFJ0S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(MFJ3S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(MFJ4S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));

	QObject::connect(RFJ0S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(RFJ3S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(RFJ4S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));

	QObject::connect(LFJ0S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(LFJ3S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(LFJ4S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));
	QObject::connect(LFJ5S, SIGNAL(boxChecked(QString, bool)),this , SLOT(slidersChecked(QString, bool)));

	if(ipd->conn == 1)
	{	
		QTimer *refresh = new QTimer(subs);
		//QTimer refresh;
		QObject::connect(this, SIGNAL(bing(QString, float)),WRJ1S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),WRJ2S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),THJ1S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),THJ2S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),THJ3S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),THJ4S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),THJ5S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),FFJ0S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),FFJ3S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),FFJ4S , SLOT(getpos(QString, float)));

		QObject::connect(this, SIGNAL(bing(QString, float)),MFJ0S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),MFJ3S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),MFJ4S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),RFJ0S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),RFJ3S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),RFJ4S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),LFJ0S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),LFJ3S , SLOT(getpos(QString, float)));
		QObject::connect(this, SIGNAL(bing(QString, float)),LFJ4S , SLOT(getpos(QString, float)));
		//QObject::connect(subs, SIGNAL(bing(QString, float)),&LFJ5S , SLOT(getpos(QString, float)));
		QObject::connect(refresh, SIGNAL(timeout()),this , SLOT(getangles()));

		refresh->start(100);
	}

		MFJ0S->getpos("MFJ0", 66.6);

	//subs->setWidget(WRJ1S);
	ui.mdiArea->addSubWindow(subs);
	subs->show();
}
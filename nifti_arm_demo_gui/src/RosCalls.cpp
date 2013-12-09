/*
 * RosCalls.cpp
 *
 *  Created on: Feb 24, 2012
 *      Author: eislector
 */

#include "RosCalls.h"

RosCalls::RosCalls() {
	// TODO Auto-generated constructor stub

}

RosCalls::~RosCalls() {
	// TODO Auto-generated destructor stub
}

QStringList* RosCalls::allRosParameters(){
	QString Call = "rosparam list";
	QByteArray output;
	rosparam.start(Call);
	rosparam.waitForReadyRead(1000);
	output = rosparam.readAll();
	parameters = QString::fromLatin1(output).split("\n");
	/*for (int i = 0; i < parameters.length(); ++i) {
		if(!parameters.at(i).startsWith(filter, Qt::CaseInsensitive)){
			parameters.removeAt(i);
		}
	}*/
	return &parameters;
}

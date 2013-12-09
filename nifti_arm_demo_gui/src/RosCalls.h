/*
 * RosCalls.h
 *
 *  Created on: Feb 24, 2012
 *      Author: eislector
 */

#ifndef ROSCALLS_H_
#define ROSCALLS_H_
#include <qobject.h>
#include <qprocess.h>
#include <qmessagebox.h>

class RosCalls : public QObject {
	Q_OBJECT
public:
	RosCalls();
	virtual  ~RosCalls();
	QStringList* allRosParameters();
private:
	QProcess rosparam;
	QStringList parameters;
};

#endif /* ROSCALLS_H_ */

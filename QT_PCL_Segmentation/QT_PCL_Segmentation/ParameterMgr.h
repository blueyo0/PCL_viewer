#pragma once

/************************************************
* 参数集合类
*
* 灵感来源于MeshLab的RichParameterSet类
* 目的是可以方便的进行全局化的参数修改，读取的功能
*************************************************/
#include <QtWidgets/QMainWindow>

#include <map>
#include <string>

using namespace std;

enum ParaType { intType, floatType, doubleType, wrongType };

class Value {
public:
	Value() {}
	~Value() {}
	// Value() {data.i=-1; type=ParaType::wrongType;}
	Value(int in) { data.i = in; type = ParaType::intType; }
	Value(float in) { data.f = in; type = ParaType::floatType; }
	Value(double in) { data.d = in; type = ParaType::doubleType; }

public:
	union { int i; float f; double d; } data;
	ParaType type = ParaType::wrongType;
};

class ParameterSet {
private:
	map<string, Value> data;

public:
	ParameterSet() {}
	~ParameterSet() {}
	void add(string paraName, Value paraVal) {
		this->data[paraName] = paraVal;
	}
	void add(string paraName, int paraVal) {
		this->data[paraName] = Value(paraVal);
	}
	void add(string paraName, float paraVal) {
		this->data[paraName] = Value(paraVal);
	}
	void add(string paraName, double paraVal) {
		this->data[paraName] = Value(paraVal);
	}
	Value get(string paraName) { return this->data[paraName]; }
	int getInt(string paraName) {
		if (this->data[paraName].type == ParaType::intType)
			return this->data[paraName].data.i;
		else
			return -1;
	};
	float getFloat(string paraName) {
		if (this->data[paraName].type == ParaType::floatType)
			return this->data[paraName].data.f;
		else
			return -1.0f;
	};
	double getDouble(string paraName) {
		if (this->data[paraName].type == ParaType::doubleType)
			return this->data[paraName].data.d;
		else
			return -1.0;
	};
	void clear() {
		this->data.clear();
	}
};


class ParameterMgr
{
private:
	map<string, ParameterSet> data;
	string curSubSetName = "UNKNOWN";

public:
	ParameterMgr() {}
	~ParameterMgr() {}
	void clear() {
		this->data.clear();
	}
	void addSubSet(string subSetName) {
		this->data[subSetName] = ParameterSet();
		this->curSubSetName = subSetName;
	}
	void clearSubSet(string subSetName) {
		this->data[subSetName].clear();
	}
	void setSubSet(string subSetName, ParameterSet subSet) {
		this->data[subSetName] = subSet;
		this->curSubSetName = subSetName;
	}
	ParameterSet getSubSet(string subSetName) {
		return this->data[subSetName];
		this->curSubSetName = subSetName;
	}

public:
	void setCurSubSet(string subSetName) {
		this->curSubSetName = subSetName;
	}
	string getCurSubSetName() { return this->curSubSetName; }
	void add(string subSetName, string paraName, Value paraVal) {
		this->data[subSetName].add(paraName, paraVal);
	}
	void add(string paraName, Value paraVal) {
		this->data[curSubSetName].add(paraName, paraVal);
	}
	void add(string paraName, int paraVal) {
		this->data[curSubSetName].add(paraName, paraVal);
	}
	void add(string paraName, float paraVal) {
		this->data[curSubSetName].add(paraName, paraVal);
	}
	void add(string paraName, double paraVal) {
		this->data[curSubSetName].add(paraName, paraVal);
	}
	Value get(string paraName) { return this->data[curSubSetName].get(paraName); }
	int getInt(string paraName) { return this->data[curSubSetName].getInt(paraName); }
	float getFloat(string paraName) { return this->data[curSubSetName].getFloat(paraName); }
	double getDouble(string paraName) { return this->data[curSubSetName].getDouble(paraName); }
};





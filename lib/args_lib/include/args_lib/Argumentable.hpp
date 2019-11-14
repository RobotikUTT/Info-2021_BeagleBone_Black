#ifndef ARGUMENTABLE_HPP
#define ARGUMENTABLE_HPP

#include "args_lib/Argument.h"

#include <map>
#include <vector>
#include <string>

using args_lib::Argument;

using std::string;
using std::vector;
using std::map;

class Argumentable {
public:
	string get(string name) const;
	long getLong(string name, long defaultValue = 0) const;
	double getDouble(string name, double defaultValue = 0) const;
	string getString(string name, string defaultValue = "") const;
	
	void setLong(string name, long value);
	void setDouble(string name, double value);
	void setString(string name, string value);

	bool has(string name) const;
	bool hasLong(string name) const;
	bool hasDouble(string name) const;
	bool hasString(string name) const;

	void reset();

	void fromList(vector<Argument> args, bool reset = false);
	vector<Argument> toList() const;
private:
	map<string, string> values;
};

#endif
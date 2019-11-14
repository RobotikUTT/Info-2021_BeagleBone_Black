#include "args_lib/Argumentable.hpp"

string Argumentable::get(string name) const {
	auto pos = this->values.find(name);

	if (pos == this->values.end()) {
		throw "not found";
	} else {
		return pos->second;
	}
}

long Argumentable::getLong(string name, long defaultValue /*= 0*/) const {
	if (!this->has(name)) {
		return defaultValue;
	}

	return stol(this->get(name));
}

double Argumentable::getDouble(string name, double defaultValue /*= 0*/) const {
	if (!this->has(name)) {
		return defaultValue;
	}
	
	return stod(this->get(name));
}

string Argumentable::getString(string name, string defaultValue /*= ""*/) const {
	if (!this->has(name)) {
		return defaultValue;
	}

	return this->get(name);
}

void Argumentable::setLong(string name, long value) {
	this->values[name] = std::to_string(value);
}

void Argumentable::setDouble(string name, double value) {
	this->values[name] = std::to_string(value);
}

void Argumentable::setString(string name, string value) {
	this->values[name] = value;
}

bool Argumentable::has(string name) const {
	auto pos = this->values.find(name);

	return pos != this->values.end();
}

bool Argumentable::hasLong(string name) const {
	if (has(name)) {
		try {
			stol(get(name));
		} catch (const std::invalid_argument& ia) {
			return false;	
		}

		return true;
	}
	return false;
}

bool Argumentable::hasDouble(string name) const {
	if (has(name)) {
		try {
			stod(get(name));
		} catch (const std::invalid_argument& ia) {
			return false;	
		}
		
		return true;
	}
	return false;
}

bool Argumentable::hasString(string name) const {
	return has(name);
}

void Argumentable::reset() {
	this->values.clear();
}

void Argumentable::fromList(vector<Argument> args, bool reset /*= false*/) {
	if (reset) {
		this->values.clear();
	}

	// Fill map with values
	for (const auto& next : args) {
		this->values[next.name] = next.value;
	}
}

vector<Argument> Argumentable::toList() const {
	vector<Argument> list;

	for (auto const& x : this->values) {
		Argument next;
		next.name = x.first;
		next.value = x.second;

		list.push_back(next);
	}

	return list;
}

#include "pathfinder/map_storage.h"

using namespace std;

MapStorage::MapStorage(std::shared_ptr<PosConvertor> convertor) : _convertor(convertor), robotRadius(0) {}

MapStorage::Vect2DBool MapStorage::buildAllowedPositions(int width, int height)
{
	for (unsigned int line = 0; line < height; line++)
	{
		this->allowedPos.emplace_back();
		for (unsigned int column = 0; column < width; column++)
			this->allowedPos[line].push_back(true);
	}
	
	ROS_DEBUG_STREAM("MapStorage: Done, map size is " << allowedPos.front().size() << "x" << allowedPos.size());
	return this->allowedPos;
}

MapStorage::Vect2DBool& MapStorage::getAllowedPositions() {
	return this->allowedPos;
}

int MapStorage::width() const {
	if (this->allowedPos.size() == 0) return 0;
	return this->allowedPos.front().size();
}

int MapStorage::height() const {
	return this->allowedPos.size();
}

bool MapStorage::isBlocked(int x, int y) const {
	if (!this->allowedPos[y][x]) {
		return true;
	}

	ros::Time now = ros::Time::now();
	for (auto& tmpshape : this->tempShapes) {
		// Wait 7 seconds before ignoring shape
		if (now - tmpshape.appearance < ros::Duration(7)) {
			if (this->isIn(tmpshape.shape, x, y)) {
				return true;
			}
		}
	}

	return false;
}

bool MapStorage::isIn(const ai_msgs::Shape& shape, int x, int y) const {
	if (shape.type == ai_msgs::Shape::RECT) {
		int width = shape.params[0];
		int height = shape.params[1];

		return x > (shape.x - width / 2) && x < (shape.x + width / 2) +
			y > (shape.y - height / 2) && y < (shape.y + height / 2);
	}
	
	// Apply circle
	else if (shape.type == ai_msgs::Shape::CIRCLE) {
		int dx = x - shape.x;
		int dy = y - shape.y;

		return dx * dx + dy * dy < shape.params[0];
	}

	return false;
}

void MapStorage::setRobotRadius(double rad) {
	this->robotRadius = rad;
}

void MapStorage::declareShape(ai_msgs::Shape shape, bool temporary) {
	if (!temporary) {
		applyShape(shape, this->allowedPos);
	} else {
		this->tempShapes.push_back(TemporaryShape(shape));
	}
}

void MapStorage::applyShape(ai_msgs::Shape& shape, MapStorage::Vect2DBool& grid) {
	// Apply centered rect (round rect when having border)
	if (shape.type == ai_msgs::Shape::RECT) {
		if (shape.params.size() < 2) {
			ROS_ERROR_STREAM("Invalid rect shape received, missing height and width...");
			return;
		}

		int width = shape.params[0];
		int height = shape.params[1];

		// Draw all sides extended
		this->applyRect(shape.x, shape.y, width + robotRadius * 2, height, grid);
		this->applyRect(shape.x, shape.y, width, height + robotRadius * 2, grid);

		// Draw cornes
		this->applyCircle(shape.x + (width / 2), shape.y - (height / 2), robotRadius, robotRadius, grid);
		this->applyCircle(shape.x - (width / 2), shape.y - (height / 2), robotRadius, robotRadius, grid);
		this->applyCircle(shape.x + (width / 2), shape.y + (height / 2), robotRadius, robotRadius, grid);
		this->applyCircle(shape.x - (width / 2), shape.y + (height / 2), robotRadius, robotRadius, grid);
	}
	
	// Apply circle
	else if (shape.type == ai_msgs::Shape::CIRCLE) {
		if (shape.params.size() < 1) {
			ROS_ERROR_STREAM("Invalid circle shape received, missing radius...");
			return;
		}

		double radiusX = shape.params[0];
		double radiusY = radiusX;
		
		// Use ellipse width/height
		if (shape.params.size() > 1) {
			radiusY = shape.params[1];
		}

		this->applyCircle(shape.x, shape.y, robotRadius + radiusX, robotRadius + radiusY, grid);
	} else {
		ROS_ERROR_STREAM(shape.type << " shape type not recognized");
	}
}

void MapStorage::applyRect(double shape_x, double shape_y, double width, double height, Vect2DBool& grid) {
	for (int y = _convertor->internalY(shape_y - height / 2); y < _convertor->internalY(height / 2 + shape_y); y ++) {
		if (y < 0 || y >= grid.size()) continue;
		
		for (int x = _convertor->internalX(shape_x - width / 2); x < _convertor->internalX(width / 2 + shape_x); x ++) {
			if (x < 0 || x >= grid[y].size()) continue;
			grid[y][x] = false;
		}
	}
}

void MapStorage::applyCircle(double shape_x, double shape_y, double radiusX, double radiusY, Vect2DBool& grid) {
	radiusX = _convertor->internalX(radiusX);
	radiusY = _convertor->internalX(radiusY);
	
	shape_x = this->_convertor->internalX(shape_x);
	shape_y = this->_convertor->internalY(shape_y);

	for (int y = -radiusY; y < radiusY; y ++) {
		if (y + shape_y < 0 || y + shape_y >= grid.size()) continue;
		for (int x = -radiusX; x < radiusX; x ++) {
			if (x + shape_x < 0 || x + shape_x >= grid[y].size()) continue;
			// If x,y in ellipse
			if (x * x / (radiusX * radiusX) + y * y / (radiusY * radiusY) <= 1) {
				grid[y + shape_y][x + shape_x] = false;
			}
		}
	}
}

void MapStorage::display() const {
	std::stringstream res;
	for (int y = 0; y < this->allowedPos.size(); y ++) {
		res << "|";
		for (int x = 0; x < this->allowedPos[y].size(); x ++) {
			res << (this->allowedPos[y][x] ? " " : "X");
		}
		res << "|\n";
	}

	ROS_DEBUG_STREAM("[Map]\n" << res.str());
}
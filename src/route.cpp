
#include <sstream>
#include <fstream>
#include <iostream>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <algorithm>

#include "geometry.h"
#include "xmlparser.h"
#include "route.h"

using namespace GPS;

std::string Route::name() const
{
    return routeName.empty() ? "Unnamed Route" : routeName;
}

unsigned int Route::numPositions() const
{
    return (unsigned int)positions.size();
}

metres Route::totalLength() const
{
    // The total length of the Route; this is the sum of the distances between successive route points.
    return routeLength;
}

metres Route::netLength() const
{
    Position firstPosition = positions[0];
    Position lastPosition = positions[positions.size() - 1];

    if ( areSameLocation(firstPosition, lastPosition) )
    {
        return 0;
    }

    return Position::distanceBetween(firstPosition, lastPosition);
}

metres Route::totalHeightGain() const
{
    assert(! positions.empty());

    metres total = 0.0;
    for (unsigned int i = 1; i < numPositions(); ++i)
    {
        metres deltaV = positions[i].elevation() - positions[i-1].elevation();
        if (deltaV > 0.0) total += deltaV; // ignore negative height differences
    }
    return total;
}

metres Route::netHeightGain() const
{
    assert(! positions.empty());

    metres deltaV = positions.back().elevation() - positions.front().elevation();
    return std::max(deltaV,0.0); // ignore negative height differences
}

degrees Route::minLatitude() const
{
    if (positions.empty()) {
        throw std::out_of_range("Cannot get the minimum latitude of an empty route");
    }

    degrees lowestLatitude = positions[0].latitude();

    double epsilon = 0.0001;

    for (int i = 0; i < positions.size(); i++)
    {
        if ( (positions[i].latitude() - lowestLatitude) < epsilon)
        {
            lowestLatitude = positions[i].latitude();
        }
    }

    return lowestLatitude;
}

degrees Route::maxLatitude() const
{
    degrees currentMax = positions[0].latitude();

    for(int i = 0; i < positions.size(); i++){
        if(positions[i].latitude() > currentMax)
            currentMax = positions[i].latitude();
    }

    return currentMax;
}

degrees Route::minLongitude() const     //MY FUNCTION
{
    assert(! positions.empty());

    degrees minLon = positions.front().longitude();
    for (const Position& pos : positions)
    {
        minLon = std::min(minLon,pos.longitude());
    }
    return minLon;
}

degrees Route::maxLongitude() const
{
    assert(! positions.empty());

    degrees maxLon = positions.front().longitude();
    for (const Position& pos : positions)
    {
        maxLon = std::max(maxLon,pos.longitude());
    }
    return maxLon;

}

metres Route::minElevation() const
{
    assert(! positions.empty());

    degrees minEle = positions.front().elevation();
    for (const Position& pos : positions)
    {
        minEle = std::min(minEle,pos.elevation());
    }
    return minEle;
}

metres Route::maxElevation() const
{
    assert(! positions.empty());

    degrees maxEle = positions.front().elevation();
    for (const Position& pos : positions)
    {
        maxEle = std::max(maxEle,pos.elevation());
    }
    return maxEle;
}

degrees Route::maxGradient() const
{
    assert(! positions.empty());

    if (positions.size() == 1) return 0.0;

    degrees maxGrad = -halfRotation/2; // minimum possible value
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres deltaH = Position::distanceBetween(positions[i],positions[i-1]);
        metres deltaV = positions[i].elevation() - positions[i-1].elevation();
        degrees grad = radToDeg(std::atan(deltaV/deltaH));
        maxGrad = std::max(maxGrad,grad);
    }
    return maxGrad;
}

degrees Route::minGradient() const
{
    assert(! positions.empty());

    if (positions.size() == 1) return 0.0;

    degrees minGrad = halfRotation/2; // maximum possible value
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres deltaH = Position::distanceBetween(positions[i],positions[i-1]);
        metres deltaV = positions[i].elevation() - positions[i-1].elevation();
        degrees grad = radToDeg(std::atan(deltaV/deltaH));
        minGrad = std::min(minGrad,grad);
    }
    return minGrad;
}

degrees Route::steepestGradient() const
{
    assert(! positions.empty());

    if (positions.size() == 1) return 0.0;

    degrees maxGrad = -halfRotation/2; // minimum possible value
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres deltaH = Position::distanceBetween(positions[i],positions[i-1]);
        metres deltaV = positions[i].elevation() - positions[i-1].elevation();
        degrees grad = radToDeg(std::atan(deltaV/deltaH));
        maxGrad = std::max(maxGrad,std::abs(grad));
    }
    return maxGrad;
}

Position Route::operator[](unsigned int idx) const
{
    return positions.at(idx);
}

Position Route::findPosition(const std::string & soughtName) const
{
    auto nameIt = std::find(positionNames.begin(), positionNames.end(), soughtName);

    if (nameIt == positionNames.end())
    {
        throw std::out_of_range("No position with that name found in the route.");
    }
    else
    {
        return positions[std::distance(positionNames.begin(),nameIt)];
    }
}

std::string Route::findNameOf(const Position & soughtPos) const
{
    auto posIt = std::find_if(positions.begin(), positions.end(),
                              [&] (const Position& pos) {return areSameLocation(pos,soughtPos);});

    if (posIt == positions.end())
    {
        throw std::out_of_range("Position not found in route.");
    }
    else
    {
        return positionNames[std::distance(positions.begin(),posIt)];
    }
}

unsigned int Route::timesVisited(const std::string & soughtName) const
{
    unsigned int timesVisited{0};

    try{

        Position position = this->findPosition(soughtName);
        for (const auto &i: positions)
            if (areSameLocation(i, position)) timesVisited++;
        
    } catch(const std::out_of_range& e){}

    return timesVisited;
}

unsigned int Route::timesVisited(const Position & soughtPos) const
{
    unsigned int timesVisited{0};

    for (const auto &i: positions)
        if (areSameLocation(i, soughtPos)) timesVisited++;

    return timesVisited;
}

std::string Route::buildReport() const
{
    return report;
}

std::string Route::getToData(std::string& data) {
	//Iterate through all of the gpx rte and trkseg tags in the route data to get to the postions
	if (!XML::Parser::elementExists(data, "gpx"))
		throw domain_error("No 'gpx' element.");
	data = XML::Parser::getElementContent(XML::Parser::getElement(data, "gpx"));

	if (!XML::Parser::elementExists(data, "rte"))
		throw domain_error("No 'rte' element.");
	data = XML::Parser::getElementContent(XML::Parser::getElement(data, "rte"));
	
	while (XML::Parser::elementExists(data, "trkseg")) {
		std::string trkseg = XML::Parser::getElementContent(XML::Parser::getAndEraseElement(data, "trkseg"));
		XML::Parser::getAndEraseElement(trkseg, "name");
			data += trkseg;
	}
	return data;
}

std::string Route::latAndLon(std::string data, std::string type) {
	std::string newPos;
	//Check for lat and lon errors 
	if (!XML::Parser::elementExists(data, type))
		throw std::domain_error("No '" + type + "' element.");

	newPos = XML::Parser::getAndEraseElement(data, type);

	if (!XML::Parser::attributeExists(newPos, "lat"))
		throw std::domain_error("no 'lat' attribute.");
	if (!XML::Parser::attributeExists(newPos, "lon"))
		throw std::domain_error("no 'lon' attribute.");

	return newPos;
}

void Route::pushPosition(std::string pos) {
	positions.push_back(getPos(pos));

	//Push the position as long as its able to get put in at the right address.
	if (positions.size() > 1 && areSameLocation(positions.back(), positions.at(positions.size() - 2))) {
		logSS << "Position ignored: " << positions.back().toString() << std::endl;
		positions.pop_back();
	}
	else
	{
		positionNames.push_back(getName(pos));
		logSS << "Position added: " << positions.back().toString() << std::endl;
	}
}

std::string Route::getName(std::string pos) {
	//Simple return the contents of the name tag thats inside the position
	if (XML::Parser::elementExists(pos, "name")) {
		return XML::Parser::getElementContent(XML::Parser::getElement(pos, "name"));
	}
	return "";
}

Position Route::getPos(std::string pos) {
	std::string ele, lat, lon;
	lat = XML::Parser::getElementAttribute(pos, "lat");
	lon = XML::Parser::getElementAttribute(pos, "lon");
	//Check to see if it contains ele in the position and if so add it to position
	if (XML::Parser::elementExists(pos, "ele")) {
		ele = XML::Parser::getElementContent(XML::Parser::getElement(pos, "ele"));
		return Position(lat, lon, ele);
	}
	else
		return Position(lat, lon);
}

void Route::setLength() {
	metres deltaH, deltaV;
	routeLength = 0;

	//Calculate through every position the distance took and therfore calculate route length
	for (unsigned int i = 1; i < positions.size(); ++i) {
		deltaH = Position::distanceBetween(positions[i - 1], positions[i]);
		deltaV = positions[i - 1].elevation() - positions[i].elevation();
		routeLength += sqrt(pow(deltaH, 2) + pow(deltaV, 2));
	}
}

Route::Route(std::string source, bool isFileName, metres granularity)
{
    using namespace std;
    using namespace XML::Parser;
    string ele,name,temp,temp2, elemData, recentPos;
    metres deltaH,deltaV;
    ostringstream oss2;
    unsigned int num;
    this->granularity = granularity;

	//Read all the data from the file if it exists
    if (isFileName){

        ifstream fs(source);

        if (! fs.good())
        {
            throw invalid_argument("Error opening source file '" + source + "'.");
        }

        logSS<< "Source file '" << source << "' opened okay." << endl;

        while (getline(fs, temp)) {
            oss2 << temp << endl;
        }

        source = oss2.str();
    }


	//itterates through the XML tags through gpx and rte
	elemData = getToData(source);

	//If there is a name tag in the data then set the route name
    if (elementExists(source, "name")) {
        routeName = XML::Parser::getElementContent(XML::Parser::getAndEraseElement(elemData, "name"));
        logSS<< "Route name is: " << routeName << endl;
    }

	//for each position get the lat and lon and ele if available and then set it in the positions vector
	while (elementExists(elemData, "rtept")) {
		recentPos = latAndLon(elemData, "rtept");
		pushPosition(recentPos);
	}
	logSS << positions.size() << " positions added." << std::endl;

	//get and set the routes length
	setLength();

	//all of the logs that have been collected then get put into a simple report file
    report = logSS.str();
}

void Route::setGranularity(metres granularity)
{
    bool implemented = false;
    assert(implemented);
}

bool Route::areSameLocation(const Position & p1, const Position & p2) const
{
    return (Position::distanceBetween(p1,p2) < granularity);
}

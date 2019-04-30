#include <sstream>
#include <fstream>
#include <iostream>
#include <cassert>
#include <cmath>
#include <stdexcept>

#include "geometry.h"
#include "xmlparser.h"
#include "track.h"

using namespace GPS;

// Note: The implementation should exploit the relationship:
//   totalTime() == restingTime() + travellingTime()

seconds Track::totalTime() const
{
    assert(! departed.empty());
    return departed.back();
}

seconds Track::restingTime() const
{
    seconds total = 0;
    assert (arrived.size() == departed.size());
    for (unsigned int i = 0; i < arrived.size(); ++i)
    {
        total += departed[i] - arrived[i];
    }
    return total;
}

seconds Track::travellingTime() const
{
    return totalTime() - restingTime();
}

speed Track::maxSpeed() const
{
    assert( positions.size() == departed.size() && positions.size() == arrived.size() );
    if (positions.size() == 1) return 0.0;

    speed ms = 0;
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres deltaH = Position::distanceBetween(positions[i],positions[i-1]);
        metres deltaV = positions[i].elevation() - positions[i-1].elevation();
        metres distance = std::sqrt(std::pow(deltaH,2) + std::pow(deltaV,2));
        seconds time = arrived[i] - departed[i-1];
        ms = std::max(ms,distance/time);
    }
    return ms;
}

speed Track::averageSpeed(bool includeRests) const
{
    seconds time = (includeRests ? totalTime() : travellingTime());
    if (time == 0) return 0;
    else return totalLength() / time;
}

speed Track::maxRateOfAscent() const
{
    assert( positions.size() == departed.size() && positions.size() == arrived.size() );
    if (positions.size() == 1) return 0.0;

    speed ms = 0;
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres height = positions[i].elevation() - positions[i-1].elevation();
        seconds time = arrived[i] - departed[i-1];
        ms = std::max(ms,height/time);
    }
    return ms;
}

speed Track::maxRateOfDescent() const
{
    assert( positions.size() == departed.size() && positions.size() == arrived.size() );
    if (positions.size() == 1) return 0.0;

    speed ms = 0;
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres height = positions[i-1].elevation() - positions[i].elevation();
        seconds time = arrived[i] - departed[i-1];
        ms = std::max(ms,height/time);
    }
    return ms;
}

void Track::chkElementExists(std::string elemName , std::string fileContent) {
    if (!XML::Parser::elementExists(fileContent, elemName)) {
        throw std::domain_error("no '" + elemName + "' element.");
    }
}

std::string Track::getElement(std::string fileContent, std::string elemName, bool erase)
{
    std::string elem;
    if(erase)
    {
        XML::Parser::getAndEraseElement(fileContent, elemName);
    }
    else
    {
        elem = XML::Parser::getElement(fileContent, elemName);
    }
    return XML::Parser::getElementContent(elem);
}
void Track::chkAttrExists(std::string attr , std::string fileContent) {
    if (! XML::Parser::attributeExists(fileContent,attr)){
        throw std::domain_error("No '" + attr + "' attribute.");
    }
}

void Track::chkLatAndLong(std::string fileContent)
{
    this->chkAttrExists("lat", fileContent);
    this->chkAttrExists("lon", fileContent);
}


Track::Track(std::string source, bool isFileName, metres granularity)
{
    // Setting main program variables
    using namespace std;
    using namespace XML::Parser;
    string mergedTrkSegs,trkseg,lat,lon,ele,name,time,temp,temp2,fileContent, elemContent;
    metres deltaH,deltaV;
    seconds startTime, currentTime, timeElapsed;
    ostringstream oss,oss2;
    unsigned int num;
    this->granularity = granularity;

    // Get source from file if content is missing
    if (isFileName) {
        ifstream fs(source);
        if (! fs.good()){
            throw invalid_argument("Error opening source file '" + source + "'.");
        }

        oss << "Source file '" << source << "' opened okay." << endl;

        //Reads through file until empty
        while (fs.good()) {
            getline(fs, temp);
            oss2 << temp << endl;
        }
        // Create Different variable for source;
        fileContent = oss2.str();
    }

    // Nest through elements getting their values

    //  TODO: Probably create separate variables for different elements
    //  Checking element exists and get its content
    this->chkElementExists("gpx", fileContent);
    fileContent = this->getElement(fileContent,"gpx",false);

    this->chkElementExists("trk", fileContent);
    fileContent = this->getElement(fileContent, "trk", false);

    // Remove name element and output value
    if (elementExists(fileContent, "name")) {
        temp = getAndEraseElement(fileContent, "name");
        routeName = getElementContent(temp);
        oss << "Track name is: " << routeName << endl;
    }

    // Iterate over segments of the track
    while (elementExists(fileContent, "trkseg")) {
        temp = getAndEraseElement(fileContent, "trkseg");
        trkseg = getElementContent(temp);
        getAndEraseElement(trkseg, "name");
        mergedTrkSegs += trkseg;
    }

    if (! mergedTrkSegs.empty()){
        source = mergedTrkSegs;
    }
    num = 0;

    this->chkElementExists("trkpt", fileContent);
    temp = getAndEraseElement(source, "trkpt");

    // Check latitude and longitude are present
    this->chkLatAndLong(fileContent);

    lat = getElementAttribute(temp, "lat");
    lon = getElementAttribute(temp, "lon");


    temp = getElementContent(temp);


    if (elementExists(temp, "ele")) {
        temp2 = getElement(temp, "ele");
        ele = getElementContent(temp2);
        Position startPos = Position(lat,lon,ele);
        positions.push_back(startPos);
        oss << "Start position added: " << startPos.toString() << endl;
        ++num;
    } else {
        Position startPos = Position(lat,lon);
        positions.push_back(startPos);
        oss << "Start position added: " << startPos.toString() << endl;
        ++num;
    }



    if (elementExists(temp,"name")) {
        temp2 = getElement(temp,"name");
        name = getElementContent(temp2);
    }


    positionNames.push_back(name);
    arrived.push_back(0);
    departed.push_back(0);

    chkElementExists("time", fileContent);

    temp2 = getElement(temp,"time");
    time = getElementContent(temp2);
    startTime = currentTime = stringToTime(time);
    Position prevPos = positions.back(), nextPos = positions.back();

    while (elementExists(source, "trkpt")) {
        temp = getAndEraseElement(source, "trkpt");

        this->chkLatAndLong(fileContent);

        lat = getElementAttribute(temp, "lat");
        lon = getElementAttribute(temp, "lon");
        temp = getElementContent(temp);

        if (elementExists(temp, "ele")) {
            temp2 = getElement(temp, "ele");
            ele = getElementContent(temp2);
            nextPos = Position(lat,lon,ele);
        } else {
            nextPos = Position(lat,lon);
        }

        chkElementExists("time", fileContent);

        temp2 = getElement(temp,"time");
        time = getElementContent(temp2);
        currentTime = stringToTime(time);

        if (areSameLocation(nextPos, prevPos)) {
            // If we're still at the same location, then we haven't departed yet.
            departed.back() = currentTime - startTime;
            oss << "Position ignored: " << nextPos.toString() << endl;
        } else {

            if (elementExists(temp,"name")) {
                temp2 = getElement(temp,"name");
                name = getElementContent(temp2);
            } else {
                name = ""; // Fixed bug by adding this.
            }


            positions.push_back(nextPos);
            positionNames.push_back(name);
            timeElapsed = currentTime - startTime;
            arrived.push_back(timeElapsed);
            departed.push_back(timeElapsed);

            oss << "Position added: " << nextPos.toString() << endl;
            oss << " at time: " << to_string(timeElapsed) << endl;
            ++num;
            prevPos = nextPos;
        }
    }


    oss << num << " positions added." << endl;
    routeLength = 0;

    for (unsigned int i = 1; i < num; ++i ) {
        deltaH = Position::distanceBetween(positions[i-1], positions[i]);
        deltaV = positions[i-1].elevation() - positions[i].elevation();
        routeLength += sqrt(pow(deltaH,2) + pow(deltaV,2));
    }

    report = oss.str();
}

void Track::setGranularity(metres granularity)
{
    bool implemented = false;
    assert(implemented);
}

seconds Track::stringToTime(const std::string & timeStr)
{
    return stoull(timeStr);
}

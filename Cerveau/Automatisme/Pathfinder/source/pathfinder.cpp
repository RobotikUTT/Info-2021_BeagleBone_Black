#include "pathfinder/pathfinder.h"

#include <chrono>
#include <sstream>
#include <utility>
#include <limits>

using namespace std;

Pathfinder::Pathfinder(std::shared_ptr<PosConvertor> convertor) : _mapStorage(convertor) {
    _mapStorage.buildAllowedPositions(200, 300);
}


int Pathfinder::findPath(const Pose2D& startPos, const Pose2D& endPos, Path& path)
{
    ROS_DEBUG_STREAM("START: " << startPos);
    ROS_DEBUG_STREAM("END: " << endPos);

    if (_mapStorage.width() == 0 || _mapStorage.height() == 0)
    {
        ROS_ERROR("Allowed positions is empty. Did you load the file?");
        return FindPath::Response::NO_PATH_FOUND;
    }

    auto startTime = chrono::high_resolution_clock::now();
    
    if (!isValid(startPos) || !isValid(endPos))
    {
        ROS_ERROR("Start or end position is not valid!");
        return FindPath::Response::START_END_POS_NOT_VALID;
    }
    
    // Creates a map filled with -1
    auto mapDist = Vect2DShort(
        _mapStorage.height(), vector<short>(_mapStorage.width(), -1)
    );

    if (!exploreGraph(mapDist, startPos, endPos)) // endPos not found or no paths exist between startPos and endPos
    {
        ROS_ERROR_STREAM("No path found !");
        return FindPath::Response::NO_PATH_FOUND;
    }
    
    Path rawPath = retrievePath(mapDist, startPos, endPos);
    path = smoothPath(rawPath);
    
    auto endTime = chrono::high_resolution_clock::now();
    chrono::duration<double, std::milli> elapsedSeconds = endTime - startTime;

    ROS_INFO_STREAM("Found a path with " << path.size() << " Pose2Ds (took " << elapsedSeconds.count() << " ms)");
    
    return FindPath::Response::PATH_FOUND;
}

Pose2D Pathfinder::getMapSize()
{
    Pose2D size;
    size.x = _mapStorage.width();
    size.y = _mapStorage.height();
    return size;
}


bool Pathfinder::exploreGraph(Vect2DShort& distMap, const Pose2D& startPos, const Pose2D& endPos)
{
    vector<Pose2D> previousPositions, nextPositions;
    short distFromEnd = 0;
    
    distMap[endPos.y][endPos.x] = distFromEnd;
    if (endPos.x == startPos.x && endPos.y == startPos.y)
        return true;
    
    previousPositions.push_back(endPos);
    distFromEnd++;
    
    while (!previousPositions.empty())
    {
        for (const Pose2D& prevPos : previousPositions)
        {
            for (const Pose2D& dir : directions())
            {
                Pose2D nextPos = add(prevPos, dir);
                if (distMap[nextPos.y][nextPos.x] == -1 && isValid(nextPos))
                {
                    distMap[nextPos.y][nextPos.x] = distFromEnd;
                    if (nextPos.x == startPos.x && nextPos.y == startPos.y) {
                        ROS_DEBUG("Goal Found!");
                        return true;
                    }
                    nextPositions.push_back(nextPos);
                }
            }
        }
        
        previousPositions.swap(nextPositions); // Swap contents not to deallocate and then reallocate memory
        nextPositions.clear();
        distFromEnd++;
    }
    
    return false; // if we reach this Pose2D, we haven't found start position
}

Pathfinder::Path Pathfinder::retrievePath(const Vect2DShort& distMap, const Pose2D& startPos, const Pose2D& endPos)
{
    Path path;
    path.push_back(startPos);
    
    Pose2D lastPos = startPos;
    
    while (lastPos.x != endPos.x && lastPos.y != endPos.y)
    {
        Pose2D bestNextPos;
        short bestDist = numeric_limits<short>::max();
        for (const Pose2D& dir : directions())
        {
            Pose2D nextPos = add(lastPos, dir);
            if (isValid(nextPos))
            {
                short posDist = distMap[nextPos.y][nextPos.x];
                if (posDist >= 0 && posDist < bestDist)
                {
                    bestDist = posDist;
                    bestNextPos = nextPos;
                }
            }
        }
        lastPos = bestNextPos;
        path.push_back(bestNextPos);
    }
    return path;
}

Pathfinder::Path Pathfinder::smoothPath(const Path& rawPath)
{
    Path newPath;
    
    newPath.push_back(rawPath.front());
    unsigned posL = 0;
    while (posL + 1 < rawPath.size())
    {
        unsigned int posR = rawPath.size() - 1; // If size() = 0 we don't enter the loop
        for (;posR > posL + 1; posR--)
            if (canConnectWithLine(rawPath[posL], rawPath[posR]))
                break;
        posL = posR; // posR was >= posL + 1, so posL[t+1] >= posL[t] + 1
        newPath.push_back(rawPath[posL]);
    }
    
    return newPath;
}


bool Pathfinder::isValid(const Pose2D& pos)
{
    if (pos.y < 0 || pos.y >= _mapStorage.height())
        return false;
    if (pos.x < 0 || pos.x >= _mapStorage.width())
        return false;

    return !_mapStorage.isBlocked(pos.x, pos.y);
}

bool Pathfinder::canConnectWithLine(const Pose2D& pA, const Pose2D& pB)
{
    Pose2D testPos = pA;
    int dX, dY, stepX, stepY, error;
    
    dX = abs(pB.x - pA.x);
    dY = abs(pB.y - pA.y);
    
    stepX = (pB.x > pA.x) ? 1 : -1;
    stepY = (pB.y > pA.y) ? 1 : -1;
    
    // We don't need to check start and end positions
    
    if (dX > dY)
    {
        error = dX/2;
        for (int i = 0; i < dX; i++)
        {
            testPos.x += stepX;
            error += dY;
            if (error > dX)
            {
                error -= dX;
                testPos.y += stepY;
            }
           if (!isValid(testPos))
               return false;
        }
    }
    else
    {
        error = dY/2;
        for (int i = 0; i < dY; i++)
        {
            testPos.y += stepY;
            error += dX;
            if (error > dY)
            {
                error -= dY;
                testPos.x += stepX;
            }
           if (!isValid(testPos))
               return false;
        }
    }
    return true;
}

Pose2D Pathfinder::add(const Pose2D pos1, const Pose2D pos2) const {
    Pose2D res;
    res.x = pos1.x + pos2.x;
    res.y = pos1.y + pos2.y;
    return res;
}

std::vector< Pose2D > Pathfinder::directions() const
{
    vector<vector<int>> numDirs {
        {0, 1},
        {1, 0},
        {0, -1},
        {-1, 0}
    };

    vector<Pose2D> dirs;
    for (auto& dir : numDirs) {
        Pose2D next;
        next.x = dir[0];
        next.y = dir[1];
        dirs.push_back(next);
    };
    return dirs; // Should use move semantics with recent compilators
}

MapStorage& Pathfinder::getMap() {
    return this->_mapStorage;
}

string Pathfinder::pathMapToStr(const Path& path)
{
    ostringstream os;
    string str = "[";
    for (const Pose2D& pos : path)
        os << pos << ", ";
    str += os.str();
    if (str.length() > 2)
        str.erase(str.end()-2, str.end());
    str += "]";
    return str;
}

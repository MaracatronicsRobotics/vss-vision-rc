#include "PositionProcessing.h"

#define INVALID_ROBOT_ID 255
#define ROBOT_0_PATTERN_COMPONENTS                                             \
  { Color::CYAN, Color::GREEN }
#define ROBOT_1_PATTERN_COMPONENTS                                             \
  { Color::PINK, Color::GREEN }
#define ROBOT_2_PATTERN_COMPONENTS                                             \
  { Color::PINK, Color::CYAN }

void PositionProcessing::saveXML() {
  cv::FileStorage file(POSITION_PROCESSING_FILE, cv::FileStorage::WRITE);
  file << "Default";
  file << "{";
  file << MINSIZE << param[MINSIZE];
  file << MAXSIZE << param[MAXSIZE];
  file << MINSIZEBALL << param[MINSIZEBALL];
  file << MAXSIZEBALL << param[MAXSIZEBALL];
  file << BLOBMAXDIST << param[BLOBMAXDIST];
  file << MYTEAM << param[MYTEAM];
  file << ENEMYTEAM << param[ENEMYTEAM];
  file << ENEMYSEARCH << param[ENEMYSEARCH];
  file << SHOWELEMENT << param[SHOWELEMENT];
  file << "}";
  file.release();
}

void PositionProcessing::matchBlobs(cv::Mat &debugFrame) {

  FieldRegions groupedBlobs = pairBlobs();
  static Players empty_players;

  vss.setPlayers(empty_players);
  // printf("Team Color %d\n", getTeamColor());

  // Settting team positions
  Players teamA;
  setTeamColor(Color::YELLOW);
  findTeam(teamA, debugFrame, groupedBlobs.team);

  Players teamB;
  setTeamColor(Color::BLUE);
  findTeam(teamB, debugFrame, groupedBlobs.team);

  Players allPlayers;
  allPlayers.insert(allPlayers.end(), teamA.begin(), teamA.end());
  allPlayers.insert(allPlayers.end(), teamB.begin(), teamB.end());
  sort(allPlayers.begin(), allPlayers.end());

  Entity ball;
  findBall(ball, debugFrame);

  vss.setEntities(ball, allPlayers);
}

void PositionProcessing::findTeam(Players &players, cv::Mat &debugFrame,
                                  std::vector<Region> &teamRegions) {
  std::map<int, std::vector<Color>> colorById;
  colorById[0] = ROBOT_0_PATTERN_COMPONENTS;
  colorById[1] = ROBOT_1_PATTERN_COMPONENTS;
  colorById[2] = ROBOT_2_PATTERN_COMPONENTS;

  players.clear();

  filterPattern(teamRegions);

  for (Region &region : teamRegions) {
    Blobs blobs = region.blobs;
    Blob b1, b2, b3;

    if (blobs.size() == 3) {
      b1 = blobs[0];
      b2 = blobs[1];
      b3 = blobs[2];
    } else {
      b1 = blobs[0];
      b2 = blobs[0];
      b3 = blobs[0];
    }

    uint8_t robotId = INVALID_ROBOT_ID;
    std::vector<int> regionColors;
    for (int i = 0; i < blobs.size(); i++) {
      regionColors.push_back(blobs[i].color);
    }

    for (std::map<int, std::vector<Color>>::iterator it = colorById.begin();
         it != colorById.end(); it++) {
      std::vector<Color> colors = it->second;
      bool found = true;
      for (int i = 0; i < colors.size(); i++) {
        if (std::find(regionColors.begin(), regionColors.end(),
                      static_cast<int>(colors.at(i))) == regionColors.end()) {
          found = false;
          break;
        }
      }
      if (found) {
        robotId = it->first;
        break;
      }
    }

    // if (robotId == INVALID_ROBOT_ID) {
    //   continue;
    // }

    Blob greatestBlob;
    for (int i = 0; i < blobs.size(); i++) {
      if (blobs.at(i).area > greatestBlob.area) {
        greatestBlob = blobs.at(i);
      }
    }

    uint teamColor = static_cast<uint>(greatestBlob.color);

    if (greatestBlob.color != getTeamColor())
      continue;

    // TODO: Implement a better way to identify the color of the robot
    Player robot(robotId); // set the id of the robot
    robot.team(teamColor);
    cv::Point secondaryPosition = (b2.position + b3.position) * 0.5;
    Point newPositionInPixels = (b1.position + secondaryPosition) * 0.5;
    Point newPosition = Utils::convertPositionPixelToCm(newPositionInPixels);

    Float newAngle = Utils::angle(b1.position, secondaryPosition);

    int firstSecondary = b2.color;
    int secondSecondary = b3.color;
    int colorIndex = firstSecondary;

    robot.update(Point(newPosition.x, newPosition.y), newAngle);
    players.push_back(robot);
    cv::circle(
        debugFrame,
        Utils::convertPositionCmToPixel(Point(newPosition.x, newPosition.y)),
        15, _colorCar[teamColor], 2, cv::LINE_AA);
    cv::circle(
        debugFrame,
        Utils::convertPositionCmToPixel(Point(newPosition.x, newPosition.y)),
        12, _colorCar[colorIndex], 2, cv::LINE_AA);
  }
}

void PositionProcessing::findBall(Entity &ball, cv::Mat &debugFrame) {
  // Desativar, so ativar quando encontrar
  // a bola no frame atual
  ball.outdate();
  Blob blobBall;
  Int maxArea = -1;
  blobBall.valid = false;

  for (size_t i = 0; i < CLUSTERSPERCOLOR; i++) {
    if (blob[OrangeCOL][i].area > maxArea && blob[OrangeCOL][i].valid) {
      blobBall = blob[OrangeCOL][i];
      maxArea = blobBall.area;
    }
  }

  if (!blobBall.valid) {
    int fps = 30;
    auto &ballPosVel = _kalmanFilterBall[0][0].update(
        this->_ballLastPosition.x, this->_ballLastPosition.y);
    Geometry::PT filtPoint(ballPosVel(0, 0), ballPosVel(1, 0));
    Geometry::PT ballVel(ballPosVel(2, 0) * fps, ballPosVel(3, 0) * fps);

    Float actualTime = vss.time().getMilliseconds();
    Float dt = (actualTime - this->_ballLastTime) / 1000; // dt in seconds
    this->_ballLastTime = actualTime;

    Float lostFrames = dt * fps;
    ballVel = ballVel / lostFrames;
    filtPoint.x = filtPoint.x + ballVel.x * dt;
    filtPoint.y = filtPoint.y + ballVel.y * dt;

    filtPoint.x = Utils::bound(filtPoint.x, -85, 85);
    filtPoint.y = Utils::bound(filtPoint.y, -65, 65);

    cv::circle(
        debugFrame,
        Utils::convertPositionCmToPixel(cv::Point(
            static_cast<int>(filtPoint.x), static_cast<int>(filtPoint.y))),
        9, _colorCar[OrangeCOL], 2, cv::LINE_AA);
    // cv::line(debugFrame,
    // Utils::convertPositionCmToPixel(cv::Point(filtPoint.x,filtPoint.y)),Utils::convertPositionCmToPixel(cv::Point(filtPoint.x+ballVel.x,filtPoint.y+ballVel.y)),_colorCar[OrangeCOL],
    // 2);

    ball.id(0);
    ball.update(Point(filtPoint.x, filtPoint.y), atan2(ballVel.y, ballVel.x));
    this->_ballLastPosition.x = filtPoint.x;
    this->_ballLastPosition.y = filtPoint.y;
  }

  // Debug
  // cv::circle(debugFrame, blobBall.position, 9, _colorCar[OrangeCOL], 2,
  // cv::LINE_AA);
  Point newPosition = Utils::convertPositionPixelToCm(blobBall.position);
  this->_ballLastPosition = vss.ball().position();

  auto &ballPosVel =
      _kalmanFilterBall[0][0].update(newPosition.x, newPosition.y);

  Geometry::PT filtPoint(ballPosVel(0, 0), ballPosVel(1, 0));
  int fps = 100;
  Geometry::PT ballVel(ballPosVel(2, 0) * fps, ballPosVel(3, 0) * fps);
  ball.update(Point(newPosition.x, newPosition.y), atan2(ballVel.y, ballVel.x));
  this->_ballLastTime = vss.time().getMilliseconds();
  ball.id(0);

  newPosition.x = Utils::bound(newPosition.x, -85, 85);
  newPosition.y = Utils::bound(newPosition.y, -65, 65);

  cv::circle(
      debugFrame,
      Utils::convertPositionCmToPixel(cv::Point(
          static_cast<int>(newPosition.x), static_cast<int>(newPosition.y))),
      9, _colorCar[OrangeCOL], 2, cv::LINE_AA);
  // cv::line(debugFrame,
  // Utils::convertPositionCmToPixel(cv::Point(filtPoint.x,filtPoint.y)),Utils::convertPositionCmToPixel(cv::Point(filtPoint.x+ballVel.x,filtPoint.y+ballVel.y)),_colorCar[OrangeCOL],
  // 2);
  this->_ballLastPosition = cv::Point(static_cast<int>(newPosition.x),
                                      static_cast<int>(newPosition.y));
}

PositionProcessing::Blobs
PositionProcessing::getNearestSecondary(Blob current) {
  Blobs choosen;

  for (int idColor = Color::RED; idColor < Color::BROWN + 1; idColor++) {
    for (int i = 0; i < CLUSTERSPERCOLOR; i++) {
      if (blob[idColor][i].valid) {
        blob[idColor][i].color = idColor;
        blob[idColor][i].distance = static_cast<int>(Utils::euclideanDistance(
            blob[idColor][i].position, current.position));

        if (blob[idColor][i].distance < blobMaxDist()) {
          if (choosen.size() == 2) {
            if (choosen.begin()->distance > blob[idColor][i].distance) {
              choosen[0] = blob[idColor][i];
              std::sort(choosen.begin(), choosen.end(),
                        [](Blob a, Blob b) { return a.distance > b.distance; });
              break;
            }
          } else {
            choosen.push_back(blob[idColor][i]);
            std::sort(choosen.begin(), choosen.end(),
                      [](Blob a, Blob b) { return a.distance > b.distance; });
            break;
          }
        }
      } else
        break;
    }
  }
  return choosen;
}

void PositionProcessing::filterPattern(Regions &regions) {

  Regions f_regions;
  // Sort regions by leftmost blob
  for (auto &r : regions) {
    if (r.blobs.size() < 3)
      continue;
    if (r.blobs[0].position.y <
        (r.blobs[1].position.y + r.blobs[2].position.y) /
            2) // Primary blob on top
    {
      if (r.blobs[1].position.x > r.blobs[2].position.x) {
        std::swap(r.blobs[1], r.blobs[2]);
      }
    } else if (r.blobs[1].position.x <
               r.blobs[2].position.x) // Primary blob on bottom
    {
      std::swap(r.blobs[1], r.blobs[2]);
    }
    f_regions.push_back(r);
  }
  regions = f_regions;
}

PositionProcessing::FieldRegions PositionProcessing::pairBlobs() {
  FieldRegions result;
  Blobs secondary;
  Region current;

  for (int teamColor = Color::BLUE; teamColor <= Color::YELLOW; teamColor++) {
    for (int i = 0; i < CLUSTERSPERCOLOR; i++) {
      if (blob[teamColor][i].valid) {
        current.blobs.clear();
        blob[teamColor][i].color = teamColor;
        secondary = this->getNearestSecondary(blob[teamColor][i]);
        if (secondary.size() == 2) {
          current.blobs.push_back(blob[teamColor][i]);
          current.blobs.push_back(secondary[0]);
          current.blobs.push_back(secondary[1]);
          current.team = teamColor;

          result.team.push_back(current);
        } else {
          current.blobs.push_back(blob[teamColor][i]);
          current.blobs.push_back(blob[teamColor][i]);
          current.blobs.push_back(blob[teamColor][i]);
          current.team = teamColor;

          result.team.push_back(current);
        }
      } else
        break;
    }
  }
  return result;
}

void PositionProcessing::setUp(std::string var, int value) {
  this->param[var] = value;

  if (var == MINSIZE) {
    this->_minSize = value;
  } else if (var == MAXSIZE) {
    this->_maxSize = value;
  } else if (var == MINSIZEBALL) {
    this->_minSizeBall = value;
  } else if (var == MAXSIZEBALL) {
    this->_maxSizeBall = value;
  } else if (var == BLOBMAXDIST) {
    this->_blobMaxDist = 30;
  } else if (var == MYTEAM) {
    this->_teamId = value;
  } else if (var == ENEMYTEAM) {
    this->_enemyTeam = value;
  } else if (var == ENEMYSEARCH) {
    this->_enemySearch = value;
  } else if (var == SHOWELEMENT) {
    this->_showElement = value;
  } else {
    spdlog::get("Vision")->warn("Variavel Invalida");
  }
}

int PositionProcessing::setColorIndex(int color, int index) {
  int sameIndex = this->_colorIndex[color];
  for (int i = RedCOL; i < ColorStrange; i++)
    if (this->_colorIndex[i] == index) {
      this->_colorIndex[i] = -1;
    }
  this->_colorIndex[color] = index;

  return sameIndex;
}

int PositionProcessing::getParam(std::string var) { return this->param[var]; }

void PositionProcessing::initBlob(PositionProcessing::Blob &blob) {
  blob.position = cv::Point(-1, -1);
  blob.area = 0;
  blob.angle = 0;
  blob.valid = false;
  blob.distance = INT_MAX;
}

void PositionProcessing::initDefault() {
  this->_colorCar[NoCOL] = cv::Scalar(255, 255, 255);
  this->_colorCar[OrangeCOL] = cv::Scalar(0, 165, 255);
  this->_colorCar[BlueCOL] = cv::Scalar(255, 0, 0);
  this->_colorCar[YellowCOL] = cv::Scalar(0, 255, 255);
  this->_colorCar[RedCOL] = cv::Scalar(0, 0, 255);
  this->_colorCar[GreenCOL] = cv::Scalar(0, 255, 0);
  this->_colorCar[PinkCOL] = cv::Scalar(255, 0, 255);
  this->_colorCar[LightBlueCOL] = cv::Scalar(210, 252, 4);
  this->_colorCar[PurpleCOL] = cv::Scalar(200, 055, 055);
  this->_colorCar[BrownCOL] = cv::Scalar(38, 66, 107);
  this->_colorCar[ColorStrange] = cv::Scalar(255, 255, 255);

  this->segColor[NoCOL] = cv::Vec3b(0, 0, 0);
  this->segColor[OrangeCOL] = cv::Vec3b(20, 147, 255);
  this->segColor[BlueCOL] = cv::Vec3b(255, 0, 0);
  this->segColor[YellowCOL] = cv::Vec3b(0, 255, 255);
  this->segColor[RedCOL] = cv::Vec3b(0, 0, 255);
  this->segColor[GreenCOL] = cv::Vec3b(0, 255, 0);
  this->segColor[PinkCOL] = cv::Vec3b(164, 0, 255);
  this->segColor[LightBlueCOL] = cv::Vec3b(206, 250, 135);
  this->segColor[PurpleCOL] = cv::Vec3b(250, 230, 230);
  this->segColor[BrownCOL] = cv::Vec3b(25, 0, 51);
  this->segColor[ColorStrange] = cv::Vec3b(0, 0, 0);

  cv::FileStorage fs(POSITION_PROCESSING_FILE, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    spdlog::get("Vision")->error("Position Processing Xml: Error diretorio, {}",
                                 POSITION_PROCESSING_FILE);
    return;
  }
  cv::FileNode node = fs[DEFAULT];
  this->setUp(MINSIZE, node[MINSIZE]);
  this->setUp(MAXSIZE, node[MAXSIZE]);
  this->setUp(MINSIZEBALL, node[MINSIZEBALL]);
  this->setUp(MAXSIZEBALL, node[MAXSIZEBALL]);
  this->setUp(BLOBMAXDIST, node[BLOBMAXDIST]);
  this->setUp(MYTEAM, node[MYTEAM]);
  this->setUp(ENEMYTEAM, node[ENEMYTEAM]);
  this->setUp(ENEMYSEARCH, node[ENEMYSEARCH]);
  this->setUp(SHOWELEMENT, node[SHOWELEMENT]);
  fs.release();
}

int PositionProcessing::getTeamColor() { return this->_teamColor; }

void PositionProcessing::setTeamColor(int teamColor) {
  this->_teamColor = teamColor;
}

int PositionProcessing::blobMaxDist() { return _blobMaxDist; }

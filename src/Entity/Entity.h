#ifndef Entity_H
#define Entity_H

#include "Utils/Types.h"

class Entity {
  uint m_id;
  Bool m_updated;
  Point m_position;
  Point m_velocity;
  Float m_angle;
  Float m_angleVelocity;
  uint m_team;

public:

  /**
   * Constructor of Entity's class that uses an default '-1' id.
   * @return An Entity object
   */
  Entity(const uint &t_id = 99);

  /**
   * Update entity's position and angle
   */
  Void update(const Point &t_position, const Point &t_velocity, const Float &t_angle = 0, const Float &t_angleVelocity = 0);

  /**
   * Get entity position
   * @return Point with the value of position in each axis
   */
  Point position();

  Point velocity();

  /**
   * Get if entity is updated or not
   * @return Boolean that is true if entity is updated or false if entity is not updated
   */
  Bool updated();

  /**
   * Set updated as false, invalidating values
   */
  Void outdate();

  /**
   * Get entity's angle
   * @return Float with the entity's angle in radians
   */
  Float angle();
  Float angleVelocity();

  /**
   * Get entity's id number
   * @return Integer with the entity's id number
   */
  const uint &id();

  /**
   * Set entity's id number
   */
  void id(const uint&);

  /**
   * Get entity's team number
   * @return Integer with the entity's team number
   */
  const uint &team();

  /**
   * Set entity's team
   */
  void team(const uint& t_team);

};

typedef Entity Player;
typedef Player Ally;
typedef Player Enemy;

using Entities      = Vector<Entity>;
using Players       = Vector<Player>;

Bool operator<(Entity& a, Entity& b);

#endif // Entity_H

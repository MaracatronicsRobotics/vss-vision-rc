#include "Entity.h"

Entity::Entity(const uint &t_id):m_id(t_id) {
}

Void Entity::outdate() {
    m_updated = false;
}

Void Entity::update(const Point &t_position, const Point &t_velocity, const Float &t_angle, const Float &t_angleVelocity) {
    m_position = t_position;
    m_velocity = t_velocity;
    m_angle = t_angle;
    m_angleVelocity = t_angleVelocity;
    m_updated = true;
}

Point Entity::position() {
    return m_position;
}

Point Entity::velocity(){
    return m_velocity;
}

Bool Entity::updated()
{
  return m_updated;
}

Float Entity::angle()
{
  return m_angle;
}

Float Entity::angleVelocity(){
    return m_angleVelocity;
}

void Entity::id(const uint& t_id) {
    m_id = t_id;
}

const uint &Entity::id()
{
  return m_id;
}

void Entity::team(const uint& t_team) {
    m_team = t_team;
}

const uint &Entity::team()
{
  return m_team;
}

Bool operator<(Entity& a, Entity& b) {
    return a.id() < b.id();
}

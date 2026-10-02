# 🏠 AI Task Planning for a Home Service Robot

**AI in Robotics (MCTR 912) · German University in Cairo · 2025 · Team 5 (4 students)**

<p align="center"><img src="images/gazebo.jpg" width="85%"></p>

📄 [Read the report (PDF)](1-%20Project%20Report/AI%20in%20Robotics%20m3.pdf)

## Summary

A simulated home robot that receives a goal — for example, "move this object to another room" — and works out the steps by itself. It combines AI planning with ROS 2 robot navigation inside a simulated apartment.

## How it works

The robot keeps a "knowledge base" of a four-room apartment (kitchen, living room, bedroom and bathroom) and eight household objects. When the user types a goal, the system writes it as a planning problem in PDDL and solves it with the Fast Downward planner, which returns a list of *move*, *pick* and *place* actions. An executor then carries out each action, sending navigation goals to the ROS 2 Nav2 stack, and updates the knowledge base after every step. Everything was demonstrated with a TurtleBot3 robot with an OpenManipulator arm in Gazebo.

## My role

✏️ *[Replace this line with 1–2 sentences about what you personally did in this project.]*

## What's in this folder

| Folder | Contents |
|---|---|
| [1- Project Report](1-%20Project%20Report/) | Milestone 3 report (PDF) |

**Team:** Bassam Walid, Styven Hany, Youssef Mohamed, Verina Maged<br>
**Tools:** ROS 2 · Nav2 · Gazebo · PDDL · Fast Downward · Python

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <dart/dart.hpp>
#include <nlohmann/json.hpp>
#include "SimCharacter.h"

using namespace dart::dynamics;

SimCharacter::SimCharacter(std::string filename)
{
    std::ifstream input(filename);
    nlohmann::json json;
    input >> json;
    skeleton = Skeleton::create(filename);
    createJoint(json, nullptr);
}

void SimCharacter::createJoint(nlohmann::json json, dart::dynamics::BodyNodePtr parent)
{
    if (json.is_object())
    {
	std::string name = json["name"];
	std::vector<double> v = json["pos"].get<std::vector<double>>();
	Eigen::Vector3d pos(v[0], v[1], v[2]);
	std::string type = json["type"];
	BodyNodePtr bn;
	if (type == "free")
	{
	    FreeJoint::Properties properties;
	    properties.mName = name;
	    properties.mT_ParentBodyToJoint.translation() = pos;
	    bn = skeleton->createJointAndBodyNodePair<FreeJoint>(parent, properties, BodyNode::AspectProperties(name)).second;
	}
	else if (type == "ball")
	{
	    BallJoint::Properties properties;
	    properties.mName = name;
	    properties.mT_ParentBodyToJoint.translation() = pos;
	    bn = skeleton->createJointAndBodyNodePair<BallJoint>(parent, properties, BodyNode::AspectProperties(name)).second;
	}
	else if (type == "planar")
	{
	    PlanarJoint::Properties properties;
	    properties.mName = name;
	    properties.mT_ParentBodyToJoint.translation() = pos;
	    std::string plane = json["plane"];
	    if (plane == "xy")
		properties.setXYPlane();
	    else if (plane == "yz")
		properties.setYZPlane();
	    else if (plane == "zx")
		properties.setZXPlane();
	    else
		std::cerr << "unknown plane: " + plane << std::endl;
	    bn = skeleton->createJointAndBodyNodePair<PlanarJoint>(parent, properties, BodyNode::AspectProperties(name)).second;
	}
	else if (type == "revolute")
	{
	    RevoluteJoint::Properties properties;
	    properties.mName = name;
	    properties.mT_ParentBodyToJoint.translation() = pos;
	    std::string axis = json["axis"];
	    if (axis == "x")
		properties.mAxis = Eigen::Vector3d::UnitX();
	    else if (axis == "y")
		properties.mAxis = Eigen::Vector3d::UnitY();
	    else if (axis == "z")
		properties.mAxis = Eigen::Vector3d::UnitZ();
	    else
		std::cerr << "unknown axis: " + axis << std::endl;
	    bn = skeleton->createJointAndBodyNodePair<RevoluteJoint>(parent, properties, BodyNode::AspectProperties(name)).second;
	}
	if (json.contains("mass"))
	    bn->setMass(json["mass"]);
	if (json.contains("COM"))
	{
	    std::vector<double> com = json["COM"].get<std::vector<double>>();
	    bn->setLocalCOM(Eigen::Vector3d(com[0], com[1], com[2]));
	}
	if (json.contains("shape"))
	{
	    for (auto &shape: json["shape"])
	    {
		std::shared_ptr<Shape> dShape;
		if (shape["type"] == "box")
		{
		    std::vector<double> size = shape["size"].get<std::vector<double>>();
		    dShape = std::make_shared<BoxShape>(Eigen::Vector3d(size[0], size[1], size[2]));
		}
		else
		    std::cerr << "unknown shape: " << shape["type"] << std::endl;
		std::vector<double> pos = shape["pos"].get<std::vector<double>>();
		Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
		tf.translation() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
		auto shapeNode = bn->createShapeNodeWith<CollisionAspect, DynamicsAspect>(dShape);
		shapeNode->setRelativeTransform(tf);
	    }
	}

	if (json.contains("children"))
	{
	    for (auto &child: json["children"])
		createJoint(child, bn);
	}
    }
}

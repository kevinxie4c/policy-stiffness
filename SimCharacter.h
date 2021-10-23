#include <string>
#include <nlohmann/json.hpp>


class SimCharacter
{
    public:
	SimCharacter(std::string filename);

	dart::dynamics::SkeletonPtr skeleton;

    private:
	void createJoint(nlohmann::json json, dart::dynamics::BodyNodePtr parent);
};

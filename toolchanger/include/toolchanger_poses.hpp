#ifndef TOOLCHANGER_POSES_H
#define TOOLCHANGER_POSES_H

#include <map>
#include <string>

namespace ToolPoses
{
std::map<std::string, double> const staging_NoTool{ { "gantry_joint_a1", -1.57086997 }, { "gantry_joint_a2", -1.22857443 },
                                                    { "gantry_joint_a3", 1.223404945 }, { "gantry_joint_a4", 0.0 },
                                                    { "gantry_joint_a5", -1.61603299 }, { "gantry_joint_a6", -1.33356401 },
                                                    { "gantry_joint_e1", 1.456291 },    { "gantry_joint_e2", 3.149507000 },
                                                    { "gantry_joint_e3", -0.7688611 } };

std::map<std::string, double> const parked{ { "gantry_joint_a1", -1.570869979 }, { "gantry_joint_a2", -1.22857443 },
                                            { "gantry_joint_a3", 1.223404945 },  { "gantry_joint_a4", 0.0 },
                                            { "gantry_joint_a5", -1.61603299 },  { "gantry_joint_a6", -1.33356401 },
                                            { "gantry_joint_e1", 1.456291 },     { "gantry_joint_e2", 3.149507000 },
                                            { "gantry_joint_e3", -0.9854187 } };

std::map<std::string, double> const hover{ { "gantry_joint_a1", -1.570869979 }, { "gantry_joint_a2", -1.22857443 },
                                           { "gantry_joint_a3", 1.223404945 },  { "gantry_joint_a4", 0.0 },
                                           { "gantry_joint_a5", -1.61603299 },  { "gantry_joint_a6", -1.33356401 },
                                           { "gantry_joint_e1", 1.456291 },     { "gantry_joint_e2", 3.149507000 },
                                           { "gantry_joint_e3", -0.9709082 } };

std::map<std::string, double> const staging_Tool{ { "gantry_joint_a1", -1.570869 }, { "gantry_joint_a2", -1.22857 },
                                                  { "gantry_joint_a3", 1.223404 },  { "gantry_joint_a4", 0.0 },
                                                  { "gantry_joint_a5", -1.61603 },  { "gantry_joint_a6", -1.33356 },
                                                  { "gantry_joint_e1", 1.193838 },  { "gantry_joint_e2", 3.149261 },
                                                  { "gantry_joint_e3", -0.97090 } };

}  // namespace

#endif

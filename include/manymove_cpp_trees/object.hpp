// src/manymove_cpp_trees/include/manymove_cpp_trees/object.hpp

#ifndef MANYMOVE_CPP_TREES_OBJECT_HPP
#define MANYMOVE_CPP_TREES_OBJECT_HPP

#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>

namespace manymove_cpp_trees
{
    /**
     * @enum ObjectActionType
     * @brief Enumerates the types of actions that can be performed on objects.
     */
    enum class ObjectActionType
    {
        ADD,        ///< Add a new object to the planning scene.
        REMOVE,     ///< Remove an existing object from the planning scene.
        ATTACH,     ///< Attach an object to a specific robot link.
        DETACH,     ///< Detach an object from a specific robot link.
        CHECK,      ///< Check if an object exists and its attachment status.
        GET_POSE    ///< Retrieve and possibly modify the pose of an object.
    };

    /**
     * @struct ObjectAction
     * @brief Represents an action to be performed on a collision object.
     */
    struct ObjectAction
    {
        ObjectActionType type;             ///< The type of object action.
        std::string object_id;             ///< Unique identifier for the object.
        
        // Parameters for ADD action
        std::string shape;                  ///< Shape type (e.g., box, mesh).
        std::vector<double> dimensions;     ///< Dimensions for primitive shapes (e.g., width, height, depth).
        geometry_msgs::msg::Pose pose;      ///< Pose of the object in the planning scene.
        std::string mesh_file;              ///< Path to the mesh file (required if shape is mesh).
        double scale_mesh_x = 1.0;          ///< Scale factor along the X-axis for mesh objects.
        double scale_mesh_y = 1.0;          ///< Scale factor along the Y-axis for mesh objects.
        double scale_mesh_z = 1.0;          ///< Scale factor along the Z-axis for mesh objects.
        
        // Parameters for ATTACH and DETACH actions
        std::string link_name;              ///< Name of the robot link to attach/detach the object.
        bool attach = true;                 ///< Flag indicating whether to attach (true) or detach (false) the object.
        
        // Parameters for GET_POSE action
        std::string first_rotation_axis;    ///< First rotation axis (e.g., "X", "Y", "Z").
        double first_rotation_rad = 0.0;    ///< First rotation angle in radians.
        std::string second_rotation_axis;   ///< Second rotation axis (e.g., "X", "Y", "Z").
        double second_rotation_rad = 0.0;   ///< Second rotation angle in radians.
        
        /**
         * @brief Default constructor.
         */
        ObjectAction() = default;
        
        /**
         * @brief Parameterized constructor for ADD action.
         * @param obj_id Unique identifier for the object.
         * @param shp Shape type.
         * @param dims Dimensions for the shape.
         * @param ps Pose of the object.
         * @param mesh Path to mesh file.
         * @param sx Scale factor X.
         * @param sy Scale factor Y.
         * @param sz Scale factor Z.
         */
        ObjectAction(const std::string &obj_id, const std::string &shp,
                    const std::vector<double> &dims, const geometry_msgs::msg::Pose &ps,
                    const std::string &mesh = "", double sx = 1.0, double sy = 1.0, double sz = 1.0)
            : type(ObjectActionType::ADD), object_id(obj_id), shape(shp),
              dimensions(dims), pose(ps), mesh_file(mesh),
              scale_mesh_x(sx), scale_mesh_y(sy), scale_mesh_z(sz) {}
        
        /**
         * @brief Parameterized constructor for ATTACH/DETACH action.
         * @param obj_id Unique identifier for the object.
         * @param link Link name.
         * @param att Attach flag (true for attach, false for detach).
         */
        ObjectAction(const std::string &obj_id, const std::string &link, bool att)
            : type(att ? ObjectActionType::ATTACH : ObjectActionType::DETACH),
              object_id(obj_id), link_name(link), attach(att) {}
        
        /**
         * @brief Parameterized constructor for CHECK action.
         * @param obj_id Unique identifier for the object.
         */
        ObjectAction(const std::string &obj_id)
            : type(ObjectActionType::CHECK), object_id(obj_id) {}
        
        /**
         * @brief Parameterized constructor for GET_POSE action.
         * @param obj_id Unique identifier for the object.
         * @param first_axis First rotation axis.
         * @param first_rad First rotation angle in radians.
         * @param second_axis Second rotation axis.
         * @param second_rad Second rotation angle in radians.
         */
        ObjectAction(const std::string &obj_id, const std::string &first_axis, double first_rad,
                    const std::string &second_axis, double second_rad)
            : type(ObjectActionType::GET_POSE), object_id(obj_id),
              first_rotation_axis(first_axis), first_rotation_rad(first_rad),
              second_rotation_axis(second_axis), second_rotation_rad(second_rad) {}
    };

    /**
     * @brief Helper function to create an ObjectAction for adding a box-shaped object.
     * @param object_id Unique identifier for the object.
     * @param dimensions Dimensions of the box (width, height, depth).
     * @param pose Pose of the object.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createAddBoxObject(const std::string &object_id, const std::vector<double> &dimensions,
                                           const geometry_msgs::msg::Pose &pose)
    {
        return ObjectAction(object_id, "box", dimensions, pose);
    }

    /**
     * @brief Helper function to create an ObjectAction for adding a mesh-shaped object.
     * @param object_id Unique identifier for the object.
     * @param pose Pose of the object.
     * @param mesh_file Path to the mesh file.
     * @param scale_x Scale factor along the X-axis.
     * @param scale_y Scale factor along the Y-axis.
     * @param scale_z Scale factor along the Z-axis.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createAddMeshObject(const std::string &object_id, const geometry_msgs::msg::Pose &pose,
                                            const std::string &mesh_file,
                                            double scale_x = 1.0, double scale_y = 1.0, double scale_z = 1.0)
    {
        return ObjectAction(object_id, "mesh", {}, pose, mesh_file, scale_x, scale_y, scale_z);
    }

    /**
     * @brief Helper function to create an ObjectAction for attaching an object.
     * @param object_id Unique identifier for the object.
     * @param link_name Name of the robot link to attach the object to.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createAttachObject(const std::string &object_id, const std::string &link_name)
    {
        return ObjectAction(object_id, link_name, true);
    }

    /**
     * @brief Helper function to create an ObjectAction for detaching an object.
     * @param object_id Unique identifier for the object.
     * @param link_name Name of the robot link to detach the object from.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createDetachObject(const std::string &object_id, const std::string &link_name)
    {
        return ObjectAction(object_id, link_name, false);
    }

    /**
     * @brief Helper function to create an ObjectAction for checking object existence.
     * @param object_id Unique identifier for the object.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createCheckObjectExists(const std::string &object_id)
    {
        return ObjectAction(object_id);
    }

    /**
     * @brief Helper function to create an ObjectAction for getting and modifying object pose.
     * @param object_id Unique identifier for the object.
     * @param first_axis First rotation axis.
     * @param first_rad First rotation angle in radians.
     * @param second_axis Second rotation axis.
     * @param second_rad Second rotation angle in radians.
     * @return Configured ObjectAction.
     */
    inline ObjectAction createGetObjectPose(const std::string &object_id, const std::string &first_axis, double first_rad,
                                           const std::string &second_axis, double second_rad)
    {
        return ObjectAction(object_id, first_axis, first_rad, second_axis, second_rad);
    }

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_OBJECT_HPP

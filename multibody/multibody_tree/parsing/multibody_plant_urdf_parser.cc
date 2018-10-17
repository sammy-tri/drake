#include "drake/multibody/multibody_tree/parsing/multibody_plant_urdf_parser.h"

#include <limits>
#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <tinyxml2.h>

#include "drake/geometry/visual_material.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/multibody/multibody_tree/parsing/package_map.h"
#include "drake/multibody/multibody_tree/parsing/parser_path_utils.h"
#include "drake/multibody/multibody_tree/parsing/tinyxml_util.h"
#include "drake/multibody/multibody_tree/parsing/urdf_geometry.h"

namespace drake {
namespace multibody {
namespace parsing {

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

using geometry::VisualMaterial;
using multibody_plant::CoulombFriction;

namespace {

const char* kWorldName = "world";

SpatialInertia<double> ParseInertial(XMLElement* node) {
  Isometry3d T = Isometry3d::Identity();

  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) {
    T = detail::OriginAttributesToTransform(origin);
  }

  double body_mass = 0;
  XMLElement* mass = node->FirstChildElement("mass");
  if (mass) {
    detail::ParseScalarAttribute(mass, "value", &body_mass);
  }

  double ixx = 0;
  double ixy = 0;
  double ixz = 0;
  double iyy = 0;
  double iyz = 0;
  double izz = 0;

  XMLElement* inertia = node->FirstChildElement("inertia");
  if (inertia) {
    detail::ParseScalarAttribute(inertia, "ixx", &ixx);
    detail::ParseScalarAttribute(inertia, "ixy", &ixy);
    detail::ParseScalarAttribute(inertia, "ixz", &ixz);
    detail::ParseScalarAttribute(inertia, "iyy", &iyy);
    detail::ParseScalarAttribute(inertia, "iyz", &iyz);
    detail::ParseScalarAttribute(inertia, "izz", &izz);
  }

  const RotationalInertia<double> rotational(ixx, iyy, izz, ixy, ixz, iyz);
  // TODO(sam.creasey) this is probably wrong and needs to be more like the
  // SDF version in ExtractSpatialInertiaAboutBoExpressedInB.
  return SpatialInertia<double>::MakeFromCentralInertia(
      body_mass, T.translation(), rotational.ReExpress(T.linear()));
}

bool ParseBody(const PackageMap& package_map,
               const std::string& root_dir,
               ModelInstanceIndex model_instance,
               XMLElement* node,
               detail::MaterialMap* materials,
               multibody_plant::MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && (std::strcmp(attr, "true") == 0)) return false;

  attr = node->Attribute("name");
  if (!attr) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": "
        "ERROR: link tag is missing name attribute.");
  }

  std::string body_name = attr;
  if (body_name == kWorldName) {
    return false;
  }

  SpatialInertia<double> inertia;
  XMLElement* inertial_node = node->FirstChildElement("inertial");
  if (!inertial_node) {
    inertia = SpatialInertia<double>(
        0, Eigen::Vector3d::Zero(), UnitInertia<double>(0, 0, 0));
  } else {
    inertia = ParseInertial(inertial_node);
  }

  // Add a rigid body to model each link.
  const RigidBody<double>& body =
      plant->AddRigidBody(body_name, model_instance, inertia);

  if (scene_graph != nullptr) {
    for (XMLElement* visual_node = node->FirstChildElement("visual");
         visual_node;
         visual_node = visual_node->NextSiblingElement("visual")) {
      geometry::GeometryInstance geometry_instance =
          detail::ParseVisual(body_name, package_map, root_dir,
                              visual_node, materials);
      plant->RegisterVisualGeometry(
          body, geometry_instance.pose(), geometry_instance.shape(),
          geometry_instance.name(), geometry_instance.visual_material(),
          scene_graph);
    }

    for (XMLElement* collision_node = node->FirstChildElement("collision");
         collision_node;
         collision_node = collision_node->NextSiblingElement("collision")) {
      CoulombFriction<double> friction;
      geometry::GeometryInstance geometry_instance =
          detail::ParseCollision(body_name, package_map, root_dir,
                                 collision_node, &friction);
      plant->RegisterCollisionGeometry(
          body, geometry_instance.pose(), geometry_instance.shape(),
          geometry_instance.name(), friction, scene_graph);
    }
  }

  return true;
}

/**
 * Parses a joint URDF specification to obtain the names of the joint, parent
 * link, child link, and the joint type. An exception is thrown if any of these
 * names cannot be determined.
 *
 * @param[in] node The XML node parsing the URDF joint description.
 * @param[out] name A reference to a string where the name of the joint
 * should be saved.
 * @param[out] type A reference to a string where the joint type should be
 * saved.
 * @param[out] parent_link_name A reference to a string where the name of the
 * parent link should be saved.
 * @param[out] child_link_name A reference to a string where the name of the
 * child link should be saved.
 */
void ParseJointKeyParams(XMLElement* node,
                         std::string* name,
                         std::string* type,
                         std::string* parent_link_name,
                         std::string* child_link_name) {
  // Obtains the joint's name.
  const char* attr = node->Attribute("name");
  if (!attr) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ +
        ": ERROR: joint tag is missing name attribute");
  }
  *name = std::string(attr);

  // Obtains the joint's type.
  attr = node->Attribute("type");
  if (!attr) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: joint " +
        *name + " is missing type attribute");
  }
  *type = std::string(attr);

  // Obtains the name of the joint's parent link.
  XMLElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: joint " +
        *name + " doesn't have a parent node!");
  }
  attr = parent_node->Attribute("link");
  if (!attr) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: joint " +
        *name + "'s parent does not have a link attribute!");
  }
  *parent_link_name = std::string(attr);

  // Obtains the name of the joint's child link.
  XMLElement* child_node = node->FirstChildElement("child");
  if (!child_node) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: joint " +
        *name + " doesn't have a child node");
  }
  attr = child_node->Attribute("link");
  if (!attr) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: joint " +
        *name + "'s child does not have a link attribute");
  }
  *child_link_name = std::string(attr);
}

void ParseJointLimits(XMLElement* node, double* lower, double* upper) {
  *lower = -std::numeric_limits<double>::infinity();
  *upper = std::numeric_limits<double>::infinity();

  XMLElement* limit_node = node->FirstChildElement("limit");
  if (limit_node) {
    detail::ParseScalarAttribute(limit_node, "lower", lower);
    detail::ParseScalarAttribute(limit_node, "upper", upper);
  }
}

void ParseJointDynamics(const std::string& joint_name,
                        XMLElement* node, double* damping) {
  *damping = 0.0;
  double coulomb_friction = 0.0;
  double coulomb_window = std::numeric_limits<double>::epsilon();

  XMLElement* dynamics_node = node->FirstChildElement("dynamics");
  if (dynamics_node) {
    detail::ParseScalarAttribute(dynamics_node, "damping", damping);
    if (detail::ParseScalarAttribute(dynamics_node, "friction",
                                     &coulomb_friction) &&
        coulomb_friction != 0.0) {
      drake::log()->warn("Joint {} specifies non-zero friction, which is "
                         "not supported by MultibodyPlant", joint_name);
    }
    if (detail::ParseScalarAttribute(dynamics_node, "coulomb_window",
                                     &coulomb_window) &&
        coulomb_window != std::numeric_limits<double>::epsilon()) {
      drake::log()->warn("Joint {} specifies non-zero coulomb_window, which is "
                         "not supported by MultibodyPlant", joint_name);
    }
  }
}

const Body<double>& GetBodyForElement(
    const std::string& element_name,
    const std::string& link_name,
    ModelInstanceIndex model_instance,
    multibody_plant::MultibodyPlant<double>* plant) {
  if (link_name == kWorldName) {
    return plant->world_body();
  }

  if (!plant->HasBodyNamed(link_name, model_instance)) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: Could not find "
        "link named \"" + link_name + "\" with model instance ID " +
        std::to_string(model_instance) + " for element " + element_name + ".");
  }
  return plant->GetBodyByName(link_name, model_instance);
}

void ParseJoint(ModelInstanceIndex model_instance,
                XMLElement* node,
                multibody_plant::MultibodyPlant<double>* plant) {
  const char* attr = node->Attribute("drake_ignore");
  if (attr && (std::strcmp(attr, "true") == 0)) return;

  // Parses the parent and child link names.
  std::string name, type, parent_name, child_name;
  ParseJointKeyParams(node, &name, &type, &parent_name, &child_name);

  const Body<double>& parent_body = GetBodyForElement(
      name, parent_name, model_instance, plant);
  const Body<double>& child_body = GetBodyForElement(
      name, child_name, model_instance, plant);

  Isometry3d transform_to_parent_body = Isometry3d::Identity();
  XMLElement* origin = node->FirstChildElement("origin");
  if (origin) {
    transform_to_parent_body = detail::OriginAttributesToTransform(origin);
  }

  Vector3d axis(1, 0, 0);
  XMLElement* axis_node = node->FirstChildElement("axis");
  if (axis_node && type.compare("fixed") != 0 &&
      type.compare("floating") != 0 && type.compare("ball") != 0) {
    detail::ParseVectorAttribute(axis_node, "xyz", &axis);
    if (axis.norm() < 1e-8) {
      throw std::runtime_error(
          std::string(__FILE__) + ": " + __func__ + ": ERROR: axis "
          "is zero.  don't do that");
    }
    axis.normalize();
  }

  // These are only used by some joint types.
  double upper = 0;
  double lower = 0;
  double damping = 0;

  if (type.compare("revolute") == 0 || type.compare("continuous") == 0) {
    ParseJointLimits(node, &lower, &upper);
    ParseJointDynamics(name, node, &damping);
    plant->AddJoint<RevoluteJoint>(
        name, parent_body, transform_to_parent_body,
        child_body, nullopt, axis, lower, upper, damping);
  } else if (type.compare("fixed") == 0) {
    plant->AddJoint<WeldJoint>(name, parent_body, transform_to_parent_body,
                               child_body, nullopt,
                               Isometry3d::Identity());
  } else if (type.compare("prismatic") == 0) {
    ParseJointLimits(node, &lower, &upper);
    ParseJointDynamics(name, node, &damping);
    plant->AddJoint<PrismaticJoint>(
        name, parent_body, transform_to_parent_body,
        child_body, nullopt, axis, lower, upper, damping);
  } else if (type.compare("floating") == 0) {
    drake::log()->warn("Joint {} specified as type floating which is not "
                       "supported by MultibodyPlant.  Leaving {} as a "
                       "free body.", name, child_name);
  } else if (type.compare("ball") == 0) {
    drake::log()->warn(
        std::string(__FILE__) + ": " + __func__ + ": Warning: ball joint "
        "is not an official part of the URDF standard.");
    throw std::runtime_error("Joint " + name + " specified at type ball which "
                             "is not supported by MultibodyPlant.");
  } else {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: "
        "Unrecognized joint type: " + type);
  }
}

void ParseTransmission(ModelInstanceIndex model_instance,
                       XMLElement* node,
                       multibody_plant::MultibodyPlant<double>* plant) {
  // Determines the transmission type.
  const char* attr = nullptr;
  XMLElement* type_node = node->FirstChildElement("type");
  if (type_node) {
    attr = type_node->GetText();
  }

  if (!attr) {
    // Old URDF format, kept for convenience
    attr = node->Attribute("type");
    if (!attr) {
      throw std::logic_error(
          std::string(__FILE__) + ": " + __func__ + ": ERROR: "
          "Transmission element is missing the type child.");
    }
  }
  std::string type(attr);

  // Checks if the transmission type is not SimpleTransmission. If it is not,
  // print a warning and then abort this method call since only simple
  // transmissions are supported at this time.
  if (type.find("SimpleTransmission") == std::string::npos) {
    drake::log()->warn(
        std::string(__FILE__) + ": " + __func__ + ": WARNING: Only "
        "SimpleTransmissions are supported right now.  This element will be "
        "skipped.");
    return;
  }

  // Determines the actuator's name.
  XMLElement* actuator_node = node->FirstChildElement("actuator");
  if (!actuator_node || !actuator_node->Attribute("name")) {
    throw std::logic_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: "
        "Transmission is missing an actuator element.");
  }
  std::string actuator_name(actuator_node->Attribute("name"));

  // Determines the name of the joint to which the actuator is attached.
  XMLElement* joint_node = node->FirstChildElement("joint");
  if (!joint_node || !joint_node->Attribute("name")) {
    throw std::logic_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: "
        "Transmission is missing a joint element.");
  }
  std::string joint_name(joint_node->Attribute("name"));

  if (!plant->HasJointNamed(joint_name, model_instance)) {
    throw std::logic_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: "
        "Transmission specifies joint " + joint_name + " which does not "
        "exist.");
  }
  const Joint<double>& joint = plant->GetJointByName(
      joint_name, model_instance);

  // Checks if the actuator is attached to a fixed joint. If so, abort this
  // method call.
  if (joint.num_positions() == 0) {
    drake::log()->warn(
        std::string(__FILE__) + ": " + __func__ + ": WARNING: Skipping "
        "transmission since it's attached to a fixed joint \"" +
        joint_name + "\".");
    return;
  }

  plant->AddJointActuator(actuator_name, joint);
}

void ParseFrame(ModelInstanceIndex model_instance,
                XMLElement* node,
                multibody_plant::MultibodyPlant<double>* plant) {
  const char* attr = node->Attribute("name");
  if (!attr) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR parsing "
        "frame name.");
  }
  std::string name = attr;

  attr = node->Attribute("link");
  if (!attr) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR missing "
        "link name for frame " + name + ".");
  }
  std::string body_name = attr;

  const Body<double>& body =
      GetBodyForElement(name, body_name, model_instance, plant);

  Isometry3d T = detail::OriginAttributesToTransform(node);
  plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
      name, body.body_frame(), T));
}

ModelInstanceIndex ParseUrdf(
    const std::string& model_name_in,
    const PackageMap& package_map,
    const std::string& root_dir,
    XMLDocument* xml_doc,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {

  XMLElement* node = xml_doc->FirstChildElement("robot");
  if (!node) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: "
        "URDF does not contain a robot tag.");
  }

  std::string model_name = model_name_in;
  if (model_name.empty()) {
    if (!node->Attribute("name")) {
      throw std::runtime_error(
          std::string(__FILE__) + ": " + __func__ + ": ERROR: Your "
          "robot must have a name attribute or a model name "
          "must be specified.");
    }
    model_name = node->Attribute("name");
  }

  const ModelInstanceIndex model_instance =
      plant->AddModelInstance(model_name);

  // Parses the model's material elements. Throws an exception if there's a
  // material name clash regardless of whether the associated RGBA values are
  // the same.
  detail::MaterialMap materials;
  for (XMLElement* material_node = node->FirstChildElement("material");
       material_node;
       material_node = material_node->NextSiblingElement("material")) {
    detail::ParseMaterial(material_node, &materials);
  }

  // Parses the model's link elements.
  for (XMLElement* link_node = node->FirstChildElement("link");
       link_node;
       link_node = link_node->NextSiblingElement("link")) {
    if (!ParseBody(package_map, root_dir, model_instance, link_node,
                   &materials, plant, scene_graph)) {
      // Determines whether the link was not parsed because it is a world link.
      const char* name_attr = link_node->Attribute("name");
      if (!name_attr) {
        throw std::runtime_error(
            std::string(__FILE__) + ": " + __func__ + ": ERROR: "
            "link tag is missing name attribute");
      }
    }
  }

#if 0
  // Parses the collision filter groups.
  for (XMLElement* group_node =
           node->FirstChildElement("collision_filter_group");
       group_node;
       group_node = group_node->NextSiblingElement("collision_filter_group")) {
    ParseCollisionFilterGroup(tree, group_node, model_instance_id);
  }
#endif

  // Parses the model's joint elements.
  for (XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
       joint_node = joint_node->NextSiblingElement("joint")) {
    ParseJoint(model_instance, joint_node, plant);
  }

  // Parses the model's transmission elements.
  for (XMLElement* transmission_node = node->FirstChildElement("transmission");
       transmission_node;
       transmission_node =
           transmission_node->NextSiblingElement("transmission")) {
    ParseTransmission(model_instance, transmission_node, plant);
  }

  if (node->FirstChildElement("loop_joint")) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": ERROR: "
        "loop joints are not supported in MultibodyTree");
  }
  // Parses the model's Drake frame elements.
  for (XMLElement* frame_node = node->FirstChildElement("frame"); frame_node;
       frame_node = frame_node->NextSiblingElement("frame"))
    ParseFrame(model_instance, frame_node, plant);

  return model_instance;
}

}  // namespace

ModelInstanceIndex AddModelFromUrdfFile(
    const std::string& file_name,
    const std::string& model_name_in,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());

  const std::string full_path = parsing::GetFullPath(file_name);

  // Opens the URDF file and feeds it into the XML parser.
  XMLDocument xml_doc;
  xml_doc.LoadFile(full_path.c_str());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error(
        std::string(__FILE__) + ": " + __func__ + ": Failed to "
        "parse XML in file " + full_path + "\n" + xml_doc.ErrorName());
  }

  // Uses the directory holding the URDF to be the root directory
  // in which to search for files referenced within the URDF file.
  std::string root_dir = ".";
  size_t found = full_path.find_last_of("/\\");
  if (found != std::string::npos) {
    root_dir = full_path.substr(0, found);
  }

  PackageMap package_map;
  // TODO(sam.creasey) Add support for using an existing package map.
  package_map.PopulateUpstreamToDrake(full_path);

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }


  return ParseUrdf(model_name_in, package_map, root_dir,
                   &xml_doc, plant, scene_graph);
}


ModelInstanceIndex AddModelFromUrdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  return AddModelFromUrdfFile(file_name, "", plant, scene_graph);
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake

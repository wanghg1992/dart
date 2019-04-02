/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <osg/Geode>
#include <osg/Light>
#include <osg/Material>
#include <osg/ShapeDrawable>

#include "dart/gui/osg/Utils.hpp"
#include "dart/gui/osg/render/VoxelGridShapeNode.hpp"

#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/dynamics/VoxelGridShape.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

//==============================================================================
class VoxelGridShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  VoxelGridShapeGeode(
      dart::dynamics::VoxelGridShape* shape,
      ShapeFrameNode* parentShapeFrame,
      VoxelGridShapeNode* parentNode);

  void refresh();
  void extractData();

protected:
  virtual ~VoxelGridShapeGeode();

  VoxelGridShapeNode* mParentNode;
  dart::dynamics::VoxelGridShape* mVoxelGridShape;
  VoxelGridShapeDrawable* mDrawable;
};

//==============================================================================
class VoxelGridShapeDrawable : public ::osg::ShapeDrawable
{
public:
  VoxelGridShapeDrawable(
      dart::dynamics::VoxelGridShape* shape,
      dart::dynamics::VisualAspect* visualAspect,
      VoxelGridShapeGeode* parent);

  void refresh(bool firstTime);

protected:
  virtual ~VoxelGridShapeDrawable();

  dart::dynamics::VoxelGridShape* mVoxelGridShape;
  dart::dynamics::VisualAspect* mVisualAspect;
  VoxelGridShapeGeode* mParent;
};

//==============================================================================
VoxelGridShapeNode::VoxelGridShapeNode(
    std::shared_ptr<dart::dynamics::VoxelGridShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this), mVoxelGridShape(shape), mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);
}

//==============================================================================
void VoxelGridShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void VoxelGridShapeNode::extractData(bool /*firstTime*/)
{
  if (nullptr == mGeode)
  {
    mGeode = new VoxelGridShapeGeode(
        mVoxelGridShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
VoxelGridShapeNode::~VoxelGridShapeNode()
{
  // Do nothing
}

//==============================================================================
VoxelGridShapeGeode::VoxelGridShapeGeode(
    dart::dynamics::VoxelGridShape* shape,
    ShapeFrameNode* parentShapeFrame,
    VoxelGridShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, this),
    mParentNode(parentNode),
    mVoxelGridShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  extractData();
}

//==============================================================================
void VoxelGridShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void VoxelGridShapeGeode::extractData()
{
  if (nullptr == mDrawable)
  {
    mDrawable
        = new VoxelGridShapeDrawable(mVoxelGridShape, mVisualAspect, this);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
VoxelGridShapeGeode::~VoxelGridShapeGeode()
{
  // Do nothing
}

//==============================================================================
VoxelGridShapeDrawable::VoxelGridShapeDrawable(
    dart::dynamics::VoxelGridShape* shape,
    dart::dynamics::VisualAspect* visualAspect,
    VoxelGridShapeGeode* parent)
  : mVoxelGridShape(shape), mVisualAspect(visualAspect), mParent(parent)
{
  refresh(true);
}

//==============================================================================
void addBoxes(
    ::osg::CompositeShape* osg_shape,
    const octomap::OcTree* tree,
    double threashold)
{
  std::vector<std::array<double, 6>> boxes;
  boxes.reserve(tree->getNumLeafNodes());
  for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
  {
    threashold = tree->getOccupancyThres();

    if (it->getOccupancy() < threashold)
      continue;

    const auto x = it.getX();
    const auto y = it.getY();
    const auto z = it.getZ();

    const auto size = it.getSize();

    ::osg::ref_ptr<::osg::Box> osg_sphere
        = new ::osg::Box(::osg::Vec3(x, y, z), size);
    osg_shape->addChild(osg_sphere);
  }
}

//==============================================================================
void VoxelGridShapeDrawable::refresh(bool firstTime)
{
  if (mVoxelGridShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if (mVoxelGridShape->checkDataVariance(
          dart::dynamics::Shape::DYNAMIC_ELEMENTS)
      || firstTime)
  {
    ::osg::ref_ptr<::osg::CompositeShape> osg_shape
        = new ::osg::CompositeShape();

    const auto& octomap = mVoxelGridShape->getOctree();
    addBoxes(osg_shape.get(), octomap.get(), 0.75);

    setShape(osg_shape);
    dirtyDisplayList();
  }

  if (mVoxelGridShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
      || firstTime)
  {
    setColor(eigToOsgVec4(mVisualAspect->getRGBA()));
  }
}

//==============================================================================
VoxelGridShapeDrawable::~VoxelGridShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart

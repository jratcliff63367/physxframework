#ifndef IMPORT_PHYSX_DOM_H
#define IMPORT_PHYSX_DOM_H

namespace PHYSICS_DOM
{
	class PhysicsDOMDef;
}

namespace IMPORT_PHYSX_DOM
{

class ImportPhysXDOM
{
public:
	static ImportPhysXDOM *create(void);

	// Imports an XML serialized asset and converts it into a standardized PhysicsDOM
	virtual bool importPhysXDOM(const char *xmlName,		// Name of the PhysX XML file
								PHYSICS_DOM::PhysicsDOMDef &dom) = 0;	// The DOM to load it into

	virtual void release(void) = 0;
protected:
	virtual ~ImportPhysXDOM(void)
	{
	}

};

} // End of IMPORT_PHYSX_DOM namespace

#endif

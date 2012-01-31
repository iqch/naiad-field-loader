// CRT
#include <sstream>
#include <iostream>

#include <map>
#include <list>
#include <vector>

#include <string>

#include <set>

using namespace std;

// NI
#include <NbEmpReader.h>
#include <NbEmpSequenceReader.h>

#include <NbBody.h>

#include <NbFactory.h>
#include <NbEmpWriter.h>

#include <NbValueType.h>

#include <NbTile.h>

#include <em_tile.h>

using namespace Nb;
using namespace em;

// STDDEF

#ifdef LINUX
#define DLLEXPORT
#define SIZEOF_VOID_P 8
#else
#define DLLEXPORT __declspec(dllexport)
#define MAKING_DSO
#define  SESI_LITTLE_ENDIAN
#endif

// H
#include <CH/CH_Manager.h>

#include <GEO/GEO_Primitive.h>
#include <GEO/GEO_PrimPoly.h>
#include <GEO/GEO_AttributeOwner.h>
#include <GEO/GEO_AttributeHandleList.h>
#include <GEO/GEO_PrimVolume.h>

#include <GU/GU_Detail.h>
//#include <GU/GU_PrimGroupClosure.h>

#include <GU/GU_PrimPart.h>
#include <GU/GU_PrimPoly.h>
#include <GU/GU_PrimVolume.h>


#include <OBJ/OBJ_Camera.h>

#include <OP/OP_OperatorTable.h>
#include <OP/OP_OperatorTable.h>
#include <OP/OP_Director.h>

#include <PRM/PRM_Include.h>
#include <PRM/PRM_SpareData.h>

#include <ROP/ROP_Node.h>
#include <ROP/ROP_Error.h>
#include <ROP/ROP_Templates.h>

#include <SOP/SOP_Node.h>

#define STR_PARM(name, idx, vi, t) { evalString(str, name, &ifdIndirect[idx], vi, (float)t); }
#define INT_PARM(name, idx, vi, t) { return evalInt(name, &ifdIndirect[idx], vi, t); }
#define FLT_PARM(name, idx, vi, t) { return evalFloat(name, &ifdIndirect[idx], vi, t); }

//#define STR_SET(name, idx, vi, t) { setString(str, name, ifdIndirect[idx], vi, (float)t); }
//#define STR_GET(name, idx, vi, t){ evalString(str, name, &ifdIndirect[idx], vi, (float)t); }

//////////////////////////////////////////////////////////////////////////

float valatpoint(const UT_Vector3 &) { return 1.0f; };

void buildFloatField(GU_Detail* gdp, const char* name, const FieldChannel1f& ch, const TileLayout& l, GB_PrimitiveGroup* grp)
{
	GB_Attribute* nmattr = gdp->primitiveAttribs().find("name");

	UT_String grpname = grp->getName(); grpname += "."; grpname += name;
	GB_PrimitiveGroup* localgrp = gdp->newPrimitiveGroup(grpname.buffer());

	int idx = nmattr->addIndex(name);

	GB_AttributeRef nmref = gdp->findPrimAttrib(nmattr);

	float cellsize =  l.cellSize();

	for(int j=0;j<l.fineTileCount();j++)
	{
		const Nb::Tile tile(l.fineTile(j));
		Vec3f bmin, bmax;
		tile.bounds(bmin,bmax);
		Vec3f d = bmax-bmin;

		UT_Vector3 vmin(bmin[0],bmin[1],bmin[2]);
		UT_Vector3 vmax(bmax[0],bmax[1],bmax[2]);
		UT_BoundingBox b(vmin,vmax);

		int isz = b.sizeX()/cellsize+0.5;
		int jsz = b.sizeY()/cellsize+0.5;
		int ksz = b.sizeZ()/cellsize+0.5;

		float* data = new float[isz*jsz*ksz];

		UT_VoxelArrayF vox;
		vox.size(isz,jsz,ksz);

		int index = 0;

		//float _sum = 0;
		//float _vmin = 10000.0f;
		//float _vmax = -10000.0f;

		//float _amin = 1000000.f;
		float _amax = 0.0f;

		UT_Vector3 pos;

		for(int ii=0;ii<isz;ii++)
			for(int jj=0;jj<jsz;jj++)
				for(int kk=0;kk<ksz;kk++)
				{
					//UT_Vector3 pos(float(ii+0.5f)/isz,float(jj+0.5f)/jsz,float(kk+0.5f)/ksz);

					vox.indexToPos(ii,jj,kk,pos);

					Vec3f D(pos.z()*d[0],pos.y()*d[1],pos.x()*d[2]);

					float fval = ch.sampleLinear(bmin+D,l);

					//_sum+=fval;

					//if(_vmax<fval) _vmax=fval;
					//if(_vmin>fval) _vmin=fval;

					float aval = fabs(fval);
					if(_amax<aval) _amax=aval;
					//if(_amin>aval) _amin=aval;

					data[index] = fval; index++;
				};

		if(_amax < 0.000001f)
		{
			delete [] data;
			continue;
		};

		GEO_Primitive* p = GU_PrimVolume::buildFromFunction(gdp,valatpoint,b,isz,jsz,ksz);
		GEO_PrimVolume* v = dynamic_cast<GEO_PrimVolume*>(p);

		vox.extractFromFlattened(data,isz,isz*jsz);
		v->setVoxels(&vox);

		grp->add(p);
		localgrp->add(p);

		int* pidx = p->castAttribData<int>(nmref);
		*pidx = idx;

		delete [] data;
	};
};

void buildVectorField(GU_Detail* gdp, const char* name, const FieldChannel3f& ch, const TileLayout& l, GB_PrimitiveGroup* grp)
{
	GB_Attribute* nmattr = gdp->primitiveAttribs().find("name");

	UT_String grpname = grp->getName(); grpname += "."; grpname += name;
	GB_PrimitiveGroup* localgrp = gdp->newPrimitiveGroup(grpname.buffer());

	UT_String nmx = name; nmx += ".x";
	UT_String nmy = name; nmy += ".y";
	UT_String nmz = name; nmz += ".z";

	int idx = nmattr->addIndex(nmx.buffer());
	int idy = nmattr->addIndex(nmy.buffer());
	int idz = nmattr->addIndex(nmz.buffer());

	GB_AttributeRef nmref = gdp->findPrimAttrib(nmattr);

	float cellsize =  l.cellSize();

	for(int j=0;j<l.fineTileCount();j++)
	{
		const Nb::Tile tile(l.fineTile(j));
		Vec3f bmin, bmax;
		tile.bounds(bmin,bmax);
		Vec3f d = bmax-bmin;

		UT_Vector3 vmin(bmin[0],bmin[1],bmin[2]);
		UT_Vector3 vmax(bmax[0],bmax[1],bmax[2]);
		UT_BoundingBox b(vmin,vmax);

		int isz = b.sizeX()/cellsize+0.5;
		int jsz = b.sizeY()/cellsize+0.5;
		int ksz = b.sizeZ()/cellsize+0.5;

		float* datax = new float[isz*jsz*ksz];
		float* datay = new float[isz*jsz*ksz];
		float* dataz = new float[isz*jsz*ksz];

		UT_VoxelArrayF vox, voy, voz;
		vox.size(isz,jsz,ksz);
		voy.size(isz,jsz,ksz);
		voz.size(isz,jsz,ksz);

		int index = 0;

		//float x_sum = 0;
		//float x_vmin = 10000.0f;
		//float x_vmax = -10000.0f;

		//float x_amin = 1000000.f;
		float x_amax = 0.0f;

		//float y_sum = 0;
		//float y_vmin = 10000.0f;
		//float y_vmax = -10000.0f;

		//float y_amin = 1000000.f;
		float y_amax = 0.0f;

		//float z_sum = 0;
		//float z_vmin = 10000.0f;
		//float z_vmax = -10000.0f;

		//float z_amin = 1000000.f;
		float z_amax = 0.0f;

		UT_Vector3 pos;

		for(int ii=0;ii<isz;ii++)
			for(int jj=0;jj<jsz;jj++)
				for(int kk=0;kk<ksz;kk++)
				{
					//UT_Vector3 pos(float(ii+0.5f)/isz,float(jj+0.5f)/jsz,float(kk+0.5f)/ksz);

					vox.indexToPos(ii,jj,kk,pos);

					Vec3f D(pos.z()*d[0],pos.y()*d[1],pos.x()*d[2]);

					Vec3f vval = ch.sampleLinear(bmin+D,l);

					//x_sum+=vval[0]; y_sum+=vval[1]; z_sum+=vval[2];

					//if(x_vmax<vval[0]) x_vmax=vval[0];
					//if(y_vmax<vval[1]) y_vmax=vval[1];
					//if(z_vmax<vval[2]) x_vmax=vval[2];

					//if(x_vmin>vval[0]) x_vmin=vval[0];
					//if(y_vmin>vval[1]) y_vmin=vval[1];
					//if(z_vmin>vval[2]) z_vmin=vval[1];

					float xaval = fabs(vval[0]);
					float yaval = fabs(vval[1]);
					float zaval = fabs(vval[2]);

					if(x_amax<xaval) x_amax=xaval;
					if(y_amax<yaval) y_amax=yaval;
					if(z_amax<zaval) z_amax=zaval;

					//if(x_amin>xaval) x_amin=xaval;
					//if(y_amin>yaval) y_amin=yaval;
					//if(z_amin>xaval) z_amin=zaval;

					datax[index] = vval[0];
					datay[index] = vval[1];
					dataz[index] = vval[2];

					index++;
				};

		if((x_amax+y_amax+z_amax) < 0.000003f)
		{
			delete [] datax; delete [] datay; delete [] dataz;
			continue;
		};

		GEO_Primitive* px = GU_PrimVolume::buildFromFunction(gdp,valatpoint,b,isz,jsz,ksz);
		GEO_PrimVolume* vx = dynamic_cast<GEO_PrimVolume*>(px);
		GEO_Primitive* py = GU_PrimVolume::buildFromFunction(gdp,valatpoint,b,isz,jsz,ksz);
		GEO_PrimVolume* vy = dynamic_cast<GEO_PrimVolume*>(py);
		GEO_Primitive* pz = GU_PrimVolume::buildFromFunction(gdp,valatpoint,b,isz,jsz,ksz);
		GEO_PrimVolume* vz = dynamic_cast<GEO_PrimVolume*>(pz);

		vox.extractFromFlattened(datax,isz,isz*jsz);
		vx->setVoxels(&vox);

		voy.extractFromFlattened(datay,isz,isz*jsz);
		vy->setVoxels(&voy);

		voz.extractFromFlattened(dataz,isz,isz*jsz);
		vz->setVoxels(&voz);

		grp->add(px); grp->add(py); grp->add(pz);
		localgrp->add(px); localgrp->add(py); localgrp->add(pz);

		int* pidx = px->castAttribData<int>(nmref);
		*pidx = idx;

		int* pidy = py->castAttribData<int>(nmref);
		*pidy = idy;

		int* pidz = pz->castAttribData<int>(nmref);
		*pidz = idz;

		vx->setVisualization(GEO_VolumeVis::GEO_VOLUMEVIS_RAINBOW,0,1);
		vy->setVisualization(GEO_VolumeVis::GEO_VOLUMEVIS_RAINBOW,0,1);
		vz->setVisualization(GEO_VolumeVis::GEO_VOLUMEVIS_RAINBOW,0,1);

		delete [] datax; delete [] datay; delete [] dataz;
	};
};

//////////////////////////////////////////////////////////////////////////
// SOP - VOLUMETRIC LOADER
//////////////////////////////////////////////////////////////////////////

class SOP_NaiadFieldsLoader : public SOP_Node
{
private:
	static int	*indirect;

public:
	SOP_NaiadFieldsLoader(OP_Network *net, const char *name, OP_Operator *op)
		: SOP_Node(net, name, op)
	{
		if(indirect == NULL) indirect = allocIndirect(64);
	};
	virtual ~SOP_NaiadFieldsLoader() {};

	virtual OP_ERROR		 cookMySop(OP_Context &context);
	static OP_Node* creator(OP_Network *net, const char *name, OP_Operator *op);
	static PRM_Template templateList[];

	static void buildBodyMenu(void *data, PRM_Name *theMenu, int theMaxSize, const PRM_SpareData *, PRM_Parm *);
	static void buildChannelsMenu(void *data, PRM_Name *theMenu, int theMaxSize, const PRM_SpareData *, PRM_Parm *);
};

int* SOP_NaiadFieldsLoader::indirect = NULL;

OP_ERROR SOP_NaiadFieldsLoader::cookMySop(OP_Context &context)
{
	flags().timeDep = 1;

	// INITIALIZATION
	gdp->clearAndDestroy();

	float now = context.myTime;

	int frm = context.getFrame();

	if(lockInputs(context) >= UT_ERROR_ABORT) return error();

	// PARAMETERS
	UT_String file = "";
	evalString(file, "file", 0, now);

	int null = 0;

	GB_AttributeRef nmref = gdp->addPrimAttrib("name",1,GB_ATTRIB_INDEX,&null);
	if(nmref.isInvalid())
	{
		cout << "I don't know!" << endl;
		return error();
	};

	try
	{
		EmpReader sr(file.buffer(), "*");

		int bc = sr.bodyCount();

		for(int i=0;i<bc;i++)
		{
			const Nb::Body* body = sr.popBody(); //.constBody(i);//ejectBody(i);

			if(! body->matches("Field")) continue;

			const String& bn = body->longname();

			GB_PrimitiveGroup* grp = gdp->newPrimitiveGroup(bn.str().c_str());

			const FieldShape &field = body->constFieldShape();

			const TileLayout& l = body->constLayout();

			//std::vector<String> chs;

			int ccnt = field.channelCount();

			for(int i=0;i<ccnt;i++)
			{
				const Channel* ch = field.channel(i)();

				ValueBase::Type tp = ch->type();

				const char* name = ch->name().str().c_str();

				std::string nms = ch->name().str();

				//cout << "NNAME: " << ch->name() << " SSTR: " << nms << " CSTR: " << nms.c_str() << endl;

				switch(tp)
				{
				case ValueBase::FloatType :
					{
						const FieldChannel1f& ch = field.constChannel1f(i);
						buildFloatField(gdp,nms.c_str(),ch,l,grp);
					}
					break;
				//case ValueBase::Vec2fType :
				case ValueBase::Vec3fType :
					{
						const FieldChannel3f& ch = field.constChannel3f(i);
						buildVectorField(gdp,nms.c_str(),ch,l,grp);
					}
					break;
				default:
					//cout << "REJECTED!" << endl;
					continue;
				};
			};
		};
	}
	catch(std::exception& e)
	{
		cerr << "Error trying to read a body from EMP file: " << e.what() << endl;
	};

	return error();
};

OP_Node* SOP_NaiadFieldsLoader::creator(OP_Network *net, const char *name, OP_Operator *op) { return new SOP_NaiadFieldsLoader(net, name, op); }

/////////////////////////////////////////////////////////
//// ATTRIBUTE TEMPLATES
/////////////////////////////////////////////////////////

static PRM_Name theFileName("file", "File");
static PRM_Default	theFileDefault(0, "");

PRM_Template SOP_NaiadFieldsLoader::templateList[] = {
	PRM_Template(PRM_FILE, 1, &theFileName, &theFileDefault),
	PRM_Template()
};

////////////////////////////////////////
// REGISTER ALL
////////////////////////////////////////

extern "C" {
	void newSopOperator(OP_OperatorTable *table)
	{
		try
		{
			Nb::begin();
		}
		catch (...)
		{
			return;
		};

		table->addOperator(new OP_Operator(
			"naiad_fields_loader", "Naiad Body Fields",
			SOP_NaiadFieldsLoader::creator, SOP_NaiadFieldsLoader::templateList,
			0,0));
	}
}

// POST-INCLUDE
#include <UT/UT_DSOVersion.h>

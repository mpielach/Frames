/*
Autor: Mateusz Pielach
Projekt wykonywany w ramach OSAD3D
Temat: III_E4 

Wyznaczenie powierzchni pokrywy œnie¿nej, roœlinnoœci, dróg oraz ich % stosunku do pod³o¿a.
Zaznaczenie ich w warstwie danych ró¿nymi wartoœciami.

Data: 28.05.2019
Wersja: v3
*/


#include <ogx/Plugins/EasyPlugin.h>
#include <ogx/Data/Clouds/CloudHelpers.h>
#include <ogx/Data/Clouds/KNNSearchKernel.h>
#include <ogx/Data/Clouds/SphericalSearchKernel.h>
#include <ogx/Data/Primitives/PrimitiveHelpers.h>

using namespace ogx;
using namespace ogx::Data;

// Homogeneous simplification: gives quasi-constant distance between points
void CloudSimplification(Execution::Context& context, Data::ResourceID &m_node_id, double &minimal_distance)
{
	ogx::Execution::Parameters in;
	in[L"node_id"] = m_node_id;
	in[L"minimal_distance"] = minimal_distance;
	context.Execution().ExecuteAlgorithmSync(L"Clouds_Simplification.HomogeneousSimplification", in);
}

// Converting RGB to HSL, output as data layers H, S, L.
void RGB2HSL(Execution::Context& context, Data::ResourceID &m_node_id)
{
	ogx::Execution::Parameters in;
	in[L"node_id"] = m_node_id;
	context.Execution().ExecuteAlgorithmSync(L"Clouds_ColorProcessing.RGBtoHSL", in);
}

// Calculating plane fitting error
void PlaneFittingError(Execution::Context& context, Data::ResourceID &m_node_id)
{
	float radius = 1.000000;		// Arbitrary value, set a priori
	ogx::Execution::Parameters in;
	in[L"node_id"] = m_node_id;
	in[L"radius"] = radius;
	in[L"layer_name_contains_radius"] = false;
	context.Execution().ExecuteAlgorithmSync(L"Clouds_LocalApproximation.PlaneErr", in);
}

// Creating a new layer
Data::Layers::ILayer* CreateLayer(Clouds::ICloud & cloud, const wchar_t *v_sLayerName)
{
	Data::Layers::ILayer *layer;
	auto layers = cloud.FindLayers(v_sLayerName);
	// Check if there is a layer
	if (!layers.empty())
	{
		layer = layers[0];
	}
	else
	{
		layer = cloud.CreateLayer(v_sLayerName, 0); // 0 - default value
	}
	return layer;
}

// Creating a new layer in simplified cloud
Data::Layers::ILayer* CreateLayerSimplified(Clouds::ICloud * cloud, const wchar_t *v_sLayerName)
{
	Data::Layers::ILayer *layer;
	layer = cloud->CreateLayer(v_sLayerName, 0); // 0 - default value
	return layer;
}

// Creating a new cloud
Data::Clouds::ICloud * CreateCloud(Nodes::ITransTreeNode* m_node, Execution::Context& context, const wchar_t *CloudName, const wchar_t *NodeName, Data::ResourceID &child_id)
{
	auto created_elem = context.Project().ElementCreate<Data::Clouds::ICloud>();
	created_elem->Rename(CloudName);

	Data::ResourceID m_cloud_id = created_elem->GetID();
	Data::Clouds::ICloud * new_cloud = created_elem->GetData<Data::Clouds::ICloud>();

	// Attach cloud element to tree node
	auto v_child = m_node->CreateChild();
	v_child->Rename(NodeName);
	v_child->SetElement(created_elem);
	child_id = v_child->GetID();		// Store ID of element
	return new_cloud;
}

// Get values from H channel
std::vector<StoredReal> GetHValues(Clouds::ICloud & cloud, Data::Clouds::PointsRange &points_all, int points_number)
{
	auto H_layer = cloud.FindLayers(L"H");
	std::vector<StoredReal> H_values;
	H_values.reserve(points_number);
	points_all.GetLayerVals(H_values, *H_layer[0]);
	return H_values;
}

// Get values from S channel
std::vector<StoredReal> GetSValues(Clouds::ICloud & cloud, Data::Clouds::PointsRange &points_all, int points_number)
{
	auto S_layer = cloud.FindLayers(L"S");
	std::vector<StoredReal> S_values;
	S_values.reserve(points_number);
	points_all.GetLayerVals(S_values, *S_layer[0]);
	return S_values;
}

// Get values from L channel
std::vector<StoredReal> GetLValues(Clouds::ICloud & cloud, Data::Clouds::PointsRange &points_all, int points_number)
{
	auto L_layer = cloud.FindLayers(L"L");
	std::vector<StoredReal> L_values;
	L_values.reserve(points_number);
	points_all.GetLayerVals(L_values, *L_layer[0]);
	return L_values;
}

// Get plane fitting error values
std::vector<StoredReal> GetPlaneErrorValues(Clouds::ICloud & cloud, Data::Clouds::PointsRange &points_all, int points_number)
{
	auto PlaneFitError_layer = cloud.FindLayers(L"plane_fitting_err");
	std::vector<StoredReal> PlaneError;
	PlaneError.reserve(points_number);
	points_all.GetLayerVals(PlaneError, *PlaneFitError_layer[0]);
	return PlaneError;
}

// Creating a layer with height values (z)
std::vector<StoredReal> GetHeightValues(Clouds::ICloud & cloud, Data::Clouds::PointsRange &points_all, int points_number, std::vector<Data::Clouds::Point3D> &xyz)
{
	Data::Layers::ILayer *z_layer = CreateLayer(cloud, L"z value"); // Z values
	std::vector<StoredReal> z_layer_values;
	z_layer_values.reserve(points_number);
	for (int i = 0; i < points_number; i++)
	{
		z_layer_values.push_back(xyz[i].z());
	}
	points_all.SetLayerVals(z_layer_values, *z_layer);

	return z_layer_values;
}

// Calculating mean z value (height)
float MeanHeightValue(Clouds::ICloud & cloud, Data::Clouds::PointsRange &points_all, int points_number, std::vector<Data::Clouds::Point3D> &xyz)
{
	float mean_height_value = 0;
	for (int i = 0; i < points_number; i++)
	{
		mean_height_value = mean_height_value + xyz[i].z();		// Sum of 'z'
	}
	mean_height_value = mean_height_value / points_number;	// Calculate mean value of z

	return mean_height_value;
}

// Marking snow points
void FindSnow(std::vector<StoredReal> L_values, std::vector<Data::Clouds::State> &states, std::vector<Data::Clouds::State> &states_simplified, int points_number)
{
	for (int i = 0; i < points_number; i++)
	{
		if (L_values[i] >= 0.7)					// Very high lightness (white color)
		{
			states[i][10] = 1;					// Index number 10 is arbitrary
			states_simplified[i][10] = 1;		// Treated as 'snow point'
		}
	}
}

// Marking vegetation points
void FindVegetation(std::vector<StoredReal> H_values, std::vector<StoredReal> L_values, std::vector<Data::Clouds::State> &states, std::vector<Data::Clouds::State> &states_simplified, int points_number)
{
	for (int i = 0; i < points_number; i++)
	{
		if (H_values[i] < 200 && L_values[i] >= 0.2 && L_values[i] <= 0.6)		// Green color and limited lightness
		{
			states[i][11] = 1;					// Index number 11 is arbitrary
			states_simplified[i][11] = 1;		// Treated as 'vegetation point'
		}
	}
}

// Marking roads points
void FindRoads(std::vector<StoredReal> H_values, std::vector<StoredReal> L_values, std::vector<StoredReal> PlaneError, std::vector<StoredReal> z_layer_values,
	std::vector<Data::Clouds::State> &states, std::vector<Data::Clouds::State> &states_simplified, float mean_height_value, int points_number)
{
	for (int i = 0; i < points_number; i++)
	{
		if (H_values[i] >= 275 && L_values[i] >= 0.2 && L_values[i] <= 0.6 && z_layer_values[i] <= (mean_height_value + 3.7) && PlaneError[i] < 0.065)
		{
			states[i][12] = 1;
			states_simplified[i][12] = 1;
		}
	}
}

// Marking each feature with different value (color)
void SetFeaturesLayerValues(std::vector<StoredReal> &features_layer_values, std::vector<Data::Clouds::State> &states, int points_number)
{
	for (int i = 0; i < points_number; i++)
	{
		if (states[i][10] == 1)
		{
			features_layer_values.push_back(300);	// Snow points will be displayed as red
		}
		else if (states[i][11] == 1)
		{
			features_layer_values.push_back(150);	// Vegetation points will be displayed as green
		}
		else if (states[i][12] == 1)
		{
			features_layer_values.push_back(101);	// Roads will be displayed as blue
		}
		else
			features_layer_values.push_back(0);		// Undefinied points will remain violet
	}
}

// Counting snow, vegetation and roads points
void CountFeaturePoints(std::vector<Data::Clouds::State> &states_simplified, int &points_after_simplification, int &snow_points, int &vegetation_points, int &roads_points)
{
	for (auto & state : states_simplified)
	{
		if (state[31] == 0)					// Is visible
		{
			points_after_simplification++;
			if (state[10] == 1)
			{
				snow_points++;
			}
			if (state[11] == 1)
			{
				vegetation_points++;
			}
			if (state[12] == 1)
			{
				roads_points++;
			}
		}
	}
}

// Calculating area ratios
double CalculateAreaRatio(double feature_points, double all_points)
{
	double percentage = feature_points / all_points * 100.0;
	return percentage;
}

struct AreasDetection : public ogx::Plugin::EasyMethod
{
	//fields
	Nodes::ITransTreeNode* m_node;

	//parameters
	Data::ResourceID m_node_id;

	int m_neighbours_count;			// number of neighbours
	double minimal_distance;

	//constructor
	AreasDetection() : EasyMethod(L"Mateusz Pielach", L"Snow, vegetation, roads detection - IIIE4")
	{
	}

	//add input/output parameters
	virtual void DefineParameters(ParameterBank& bank)
	{
		bank.Add(L"node_id", m_node_id = Data::ResourceID::invalid).AsNode();	//cloud choice
		bank.Add(L"minimal distance", minimal_distance = 0.1).Min(0.1);
	}

	bool Init(Execution::Context& context)
	{
		OGX_SCOPE(log);
		//get node from id
		m_node = context.m_project->TransTreeFindNode(m_node_id);
		if (!m_node) ReportError(L"You must define node_id");

		OGX_LINE.Msg(User, L"Initialization succeeded");
		return EasyMethod::Init(context);
	}

	virtual void Run(Context& context)
	{
		Data::Clouds::ForEachCloud(*m_node, [&](Clouds::ICloud & cloud, Nodes::ITransTreeNode & node)
		{
			// Access points in the cloud
			Data::Clouds::PointsRange points_all;
			cloud.GetAccess().GetAllPoints(points_all);

			// Get color and xyz
			std::vector<Data::Clouds::Color> color;
			points_all.GetColors(color);
			std::vector<Data::Clouds::Point3D> xyz;
			points_all.GetXYZ(xyz);
			auto point = context.Feedback().GetFocusPoint();

			// Count all points
			int points_number = xyz.size();

			// Create new cloud which will be simplificated
			Data::ResourceID simplified_cloud_id;
			Data::Clouds::ICloud *simplified_cloud = CreateCloud(m_node, context, L"Simplified cloud", L"Simplified cloud", simplified_cloud_id);
			Clouds::PointsRange simplified_range;
			simplified_cloud->GetAccess().AllocPoints(points_all.size(), &simplified_range);
			simplified_range.SetColors(color);
			simplified_range.SetXYZ(xyz);

			CloudSimplification(context, simplified_cloud_id, minimal_distance);	// Simplification
			RGB2HSL(context, simplified_cloud_id);									// HSL values - will be used to clasify features
			RGB2HSL(context, m_node_id);
			PlaneFittingError(context, m_node_id);									// PFE - will be used to detect roads

			// Store states of points in original and simplified cloud
			std::vector<Data::Clouds::State> states;
			points_all.GetStates(states);
			std::vector<Data::Clouds::State> states_simplified;
			simplified_range.GetStates(states_simplified);

			// Create feature layers
			Data::Layers::ILayer *features_layer = CreateLayer(cloud, L"features"); 
			Data::Layers::ILayer *features_layer_simplified = CreateLayerSimplified(simplified_cloud, L"features");
			std::vector<StoredReal> features_layer_values;
			features_layer_values.reserve(xyz.size());

			std::vector<StoredReal> H_values = GetHValues(cloud, points_all, points_number);
			std::vector<StoredReal> S_values = GetSValues(cloud, points_all, points_number);
			std::vector<StoredReal> L_values = GetLValues(cloud, points_all, points_number);
			std::vector<StoredReal> PlaneError = GetPlaneErrorValues(cloud, points_all, points_number);

			// Find snow, vegetation and roads
			FindSnow(L_values, states, states_simplified, points_number);										
			FindVegetation(H_values, L_values, states, states_simplified, points_number);
			std::vector<StoredReal> z_layer_values = GetHeightValues(cloud, points_all, points_number, xyz);	// Store 'z' values (height)
			float mean_height_value = MeanHeightValue(cloud, points_all, points_number, xyz);					// Calculate mean height value
			FindRoads(H_values, L_values, PlaneError, z_layer_values, states, states_simplified, mean_height_value, points_number);

			// Set an original value for each feature
			SetFeaturesLayerValues(features_layer_values, states, points_number);
			points_all.SetLayerVals(features_layer_values, *features_layer);
			simplified_range.SetLayerVals(features_layer_values, *features_layer_simplified);

			// Area measure = number of points after cloud simplification
			int points_after_simplification = 0;
			int snow_points = 0;
			int vegetation_points = 0;
			int roads_points = 0;

			CountFeaturePoints(states_simplified, points_after_simplification, snow_points, vegetation_points, roads_points);
			points_all.SetStates(states);	// Update cloud states

			OGX_LINE.Format(ogx::Info, L"%d points in simplified cloud", points_after_simplification);
			OGX_LINE.Format(ogx::Info, L"%d points are treated as snow", snow_points);
			OGX_LINE.Format(ogx::Info, L"%d points are treated as vegetation", vegetation_points);
			OGX_LINE.Format(ogx::Info, L"%d points are treated as roads", roads_points);

			double snow_percentage = CalculateAreaRatio(snow_points, points_after_simplification);
			double vegetation_percentage = CalculateAreaRatio(vegetation_points, points_after_simplification);
			double roads_percentage = CalculateAreaRatio(roads_points, points_after_simplification);

			OGX_LINE.Format(ogx::Info, L"%f %% snow", snow_percentage);
			OGX_LINE.Format(ogx::Info, L"%f %% vegetation", vegetation_percentage);
			OGX_LINE.Format(ogx::Info, L"%f %% roads", roads_percentage);
		});
	}
};

OGX_EXPORT_METHOD(AreasDetection)
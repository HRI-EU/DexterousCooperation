// Description: API Definition to Transfer Data between HondaControl, SLAMDevice and HoloControl //

// -------------------------------------------------------- General Vocabulary -------------------------------------------------------------------------------------------//
// HondaControl := Computer (Linux) where Honda Research Institut is hosting their Robotic System and Vicon Tracking System
// HoloControl := Computer (Windows 10) where Holo-Light is hosting a control Server, which communicates with AR Devices
// SLAMDevice := Local SLAM Capable augmented reality device e.g. Microsoft HoloLens
// Hologram := 2D or 3D Feature, which will be visualized in the Field of View of the AR Device

// The Documentation is with the following Specifications
// Block Separator := &
// Parameter Separator := $
// Array Separator := ~   (was § before)
// Vector Separator := "   (was ?)

// -------------------------------------------------------- From HondaControl To SLAMDevice - Create or Change a Transform ---------------------------------------------------//
// Transform := {AddUpdateTransform & TransformParameter} 

// TransformParameters := {string ParentId & string Id & Vector3 Position & Quaternion Rotation}

// -------------------------------------------------------- From HondaControl To SLAMDevice - Create or Change a Hologram ----------------------------------------------------//
// Hologram := {AddUpdateHologram & GeneralHologramParameter & SpecificHologramType & SpecificHologramParameter}

// GeneralHologramParameters := {string ParentId $ string Id $ string Topic $ Color32 Color $ float Lifetime $ bool isSelected}

// SpecificHologramType := {Axes} 
// SpecificHologramParameters := {float scale}

// SpecificHologramType := {ArrowPlain} 
// SpecificHologramParameters := {string endId $ float length $ bool isProportional}

// SpecificHologramType := {ArrrowProgress} 
// SpecificHologramParameters := {string endId $ float length $ bool isProportional $ float progress}

// SpecificHologramType := {ArrowDirectional} 
// SpecificHologramParameters := {Vector3 Tip $ float length $ bool isProportional $ bool isWorldSpace}

// SpecificHologramType: {TrajectoryLines} 
// SpecificHologramParameters := {float animationSpeed $ bool isAlternating $ Vector3[] points $ Color32[] alternatingColors}

// SpecificHologramType: {TrajectoryArrows} 
// SpecificHologramParameters: {float animationSpeed $ bool isAlternating $ Vector3[] points $ Color32[] alternatingColors $ float arrowLength}

// SpecificHologramType: {TrajectoryDots} 
// SpecificHologramParameters: {float animationSpeed $ bool isAlternating $ Vector3[] points $ Color32[] alternatingColors}

// SpecificHologramType: {Poly2DLabel} 
// SpecificHologramParameters: {string text $ string targetId}

// SpecificHologramType: {Poly2DSprite} 
// SpecificHologramParameters: {string name $ float scale $ string targetId}

// SpecificHologramType: {Poly2DGeometry} 
// SpecificHologramParameters: {Vector2[] points $ Vector3 direction}

// SpecificHologramType: {Poly2DDropMarker} 
// SpecificHologramParameters: {}

// SpecificHologramType: {Poly3DCube} 
// SpecificHologramParameters: {Vector3 extends}

// SpecificHologramType: {Poly3DSphere} 
// SpecificHologramParameters: {float radius}

// SpecificHologramType: {Poly3DCylinder} 
// SpecificHologramParameters: {float radius $ float length}

// SpecificHologramType: {Poly3DStaticModel} 
// SpecificHologramParameters: {string modelName $ float scale}

// SpecificHologramType: {Poly3DPoseModel} 
// SpecificHologramParameters: {string modelName $ string subModelName $ float[] properties}

// Remarks:
// string := "Some String"
// float := "1234.56789"
// bool := "True" or "False"
// byte := "123"
// Vector2 := {float x ? float y}
// Vector2 := {float x ? float y}
// Vector3 := {float x ? float y ? float z}
// Quaternion := {float x ? float y ? float z ? float w}
// Color32 := {byte r ? byte g ? byte b ? byte a}
// Vector2[] := {Vector2 x1 § Vector2 x2 § Vector2 x3...}
// Vector3[] := {Vector3 x1 § Vector3 x2 § Vector3 x3...}
// Color32[] := {Color32 c1 § Color32 c2 § Color32 c3...}

// -------------------------------------------------------- From HondaControl To SLAMDevice - Clear a Hologram, Topics of Holograms, or All ------------------------------------//
// Clear Hologram := {ClearHologram & ClearParameter}
// ClearParameters: {string id, string topic}

// -------------------------------------------------------- From HondaControl To SLAMDevice - Text To Speech -------------------------------------------------------------------//
// TextToSpeech := {TextToSpeech & TextToSpeechParameter}
// TestToSpeech: {string text}

// -------------------------------------------------------- From HondaControl To SLAMDevice - SLAM Device Configuration --------------------------------------------------------//
// SLAM Device Configuration := {SLAMDeviceConfiguration & ConfigurationType & ConfigurationParameters}

// ConfigrationType = {Window}
// TestToSpeech: {bool isActive}

// -------------------------------------------------------- From SLAMDevice To HondaControl - Input ----------------------------------------------------------------------------//
// Input := {SLAMDeviceInput & int SLAMDeviceId & InputType & InputParameter}

// InputType := {Tap}
// InputParamter := {}

// InputType := {SpeechCommand}
// InputParameter := {string Command}

// -------------------------------------------------------- From SLAMDevice To HondaControl - SLAM Device Transform -------------------------------------------------------------//
// SLAM Device Transform := {ARDeviceTransform & int SLAMDeviceId & TransformParameter}
// TransformParameter := {Vector3 position $ Quaternion rotation}

// -------------------------------------------------------- From SLAMDevice To HondaControl - Vicon Tracking --------------------------------------------------------------------//
// SLAM Device Transform := {ViconTracking & int SLAMDeviceId & bool isSynchronizing}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
namespace HoloLight.Honda.API
{
    public static class Separator
    {
        public static Type Block = new Type('&');

        public static Type Parameter = new Type('$');

        public static Type Array = new Type('§');

        public static Type Vector = new Type('?');

        public static Type NetLastFrag = new Type('°');

        public static Type NetMsgSplit = new Type('|');
    }

    public class Type
    {
        private char _separator;

        public Type(char separator)
        {
            _separator = separator;
        }

        public char AsChar()
        {
            return _separator;
        }

        public string AsString()
        {
            return _separator.ToString();
        }
    }
}

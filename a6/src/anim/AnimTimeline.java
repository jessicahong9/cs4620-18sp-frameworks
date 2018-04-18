package anim;

import java.util.TreeSet;

import org.lwjgl.util.vector.Matrix;

import common.SceneObject;
import egl.math.Matrix3;
import egl.math.Matrix4;
import egl.math.Quat;
import egl.math.Vector3;

/**
 * A timeline for a particular object in the scene.  The timeline holds
 * a sequence of keyframes and a reference to the object that they
 * pertain to.  Via linear interpolation between keyframes, the timeline
 * can compute the object's transformation at any point in time.
 * 
 * @author Cristian
 */
public class AnimTimeline {
	
	/**
	 * A sorted set of keyframes.  Invariant: there is at least one keyframe.
	 */
	public final TreeSet<AnimKeyframe> frames = new TreeSet<>(AnimKeyframe.COMPARATOR);
	
	/**
	 * The object that this timeline animates
	 */
	public final SceneObject object;

	/**
	 * Create a new timeline for an object.  The new timeline initially has the object
	 * stationary, with the same transformation it currently has at all times.  This is
	 * achieve by createing a timeline with a single keyframe at time zero.
	 * @param o Object
	 */
	public AnimTimeline(SceneObject o) {
		object = o;
		
		// Create A Default Keyframe
		AnimKeyframe f = new AnimKeyframe(0);
		f.transformation.set(o.transformation);
		frames.add(f);
	}
	
	/**
	 * Add A keyframe to the timeline.
	 * @param frame Frame number
	 * @param t Transformation
	 */
	public void addKeyFrame(int frame, Matrix4 t) {
		// TODO#A6: Add an AnimKeyframe to frames and set its transformation
		
		// creates the animation frame 
		AnimKeyframe f = new AnimKeyframe(frame); 
		f.transformation.set(t);
		
		// adds the animation frame
		frames.add(f);
	}
	/**
	 * Remove a keyframe from the timeline.  If the timeline is empty,
	 * maintain the invariant by adding a single keyframe with the given
	 * transformation.
	 * @param frame Frame number
	 * @param t Transformation
	 */
	public void removeKeyFrame(int frame, Matrix4 t) {
		// TODO#A6: Delete a frame, you might want to use Treeset.remove
		// If there is no frame after deletion, add back this frame.
		
		// we create a local copy of this frame we want to remove
		AnimKeyframe f = new AnimKeyframe(frame);
		f.transformation.set(t);
		
		// we remove it 
		frames.remove(f);
		
		if (frames.isEmpty()) {
			// if there is no frame in the current animation we add this back 
			frames.add(f);
		}
		
	}

	/**
	 * Takes a rotation matrix and decomposes into Euler angles. 
	 * Returns a Vector3 containing the X, Y, and Z degrees in radians.
	 * Formulas from http://nghiaho.com/?page_id=846
	 */
	public static Vector3 eulerDecomp(Matrix3 mat) {
		double theta_x = Math.atan2(mat.get(2, 1), mat.get(2, 2));
		double theta_y = Math.atan2(-mat.get(2, 0), Math.sqrt(Math.pow(mat.get(2, 1), 2) + Math.pow(mat.get(2, 2), 2)));
		double theta_z = Math.atan2(mat.get(1, 0), mat.get(0, 0));
		
		return new Vector3((float)theta_x, (float)theta_y, (float)theta_z);
	}
	
	
	/**
	 * Update the transformation for the object connected to this timeline to the current frame
	 * @curFrame Current frame number
	 * @rotation Rotation interpolation mode: 
	 * 0 - Euler angles, 
	 * 1 - Linear interpolation of quaternions,
	 * 2 - Spherical linear interpolation of quaternions.
	 */
	public void updateTransformation(int curFrame, int rotation) {
		//TODO#A6: You need to get pair of surrounding frames,
		// calculate interpolation ratio,
		// calculate Translation, Scale and Rotation Interpolation,
		// and combine them.
		// Argument curFrame is current frame number
		// Argument rotation is rotation interpolation mode
		// 0 - Euler angles, 
		// 1 - Linear interpolation of quaternions,
		// 2 - Spherical linear interpolation of quaternions.
		
		AnimKeyframe f = new AnimKeyframe(curFrame);
		AnimKeyframe prev = frames.floor(f);
		AnimKeyframe next= frames.ceiling(f);
		
	if (frames.size() >1 && prev != null && next != null && !next.equals(prev)) {
		
			
			
			// The current interpolation
			float t = (next.frame -curFrame);
			t /= (next.frame - prev.frame);
		
			// gets the translation of both frames
			Vector3 trans0 =prev.transformation.getTrans();
			Vector3 trans1 = next.transformation.getTrans();
			
			Vector3 transout = trans0.mul(t).add(trans1.mul(1-t));
			
			Matrix3 rot_s0 = prev.transformation.getAxes();
			Matrix3 rot_s1 = next.transformation.getAxes();
			
			//creates scale and rotation matricies 
			Matrix3 scale0  = new Matrix3();
			Matrix3 scale1 = new Matrix3();
			Matrix3 scaleout = new Matrix3();
			
			Matrix3 rot0  = new Matrix3();
			Matrix3 rot1 = new Matrix3();
			
			
			rot_s0.polar_decomp(rot0, scale0);
			rot_s1.polar_decomp(rot1, scale1);
			
			// assigns the liner interpolation for scale
			scaleout.interpolate(scale0, scale1, 1-t);
	
			
			Quat quat0 = new Quat(rot0);
			Quat quat1 = new Quat(rot1);
			
			Quat out = new Quat();
			Matrix3 rotout = new Matrix3();
			Vector3 lin_angle = new Vector3(); //used in case  of linear interpolation;
			switch (rotation) {
			
 		
				case 0:
						Vector3 temp0 = eulerDecomp(rot0);
						Vector3 temp1 = eulerDecomp(rot1);
					     lin_angle = temp0.mul(t).add(temp1.mul(1-t));
						break;
				case 1:
						 out = quat0.scale(t).add(quat1.scale(1-t));
						 out.normalize();
						 
						break;
				case 2:
					
						out = Quat.slerp(quat0, quat1, 1-t);
						break;
					
				
				default:break;
			}
			
			if (rotation == 0) {
				Matrix3 rotx = Matrix3.createRotationX(lin_angle.x);
				Matrix3 roty = Matrix3.createRotationY(lin_angle.y);
				Matrix3 rotz = Matrix3.createRotationZ(lin_angle.z);
				
				rotout = rotz.mulBefore(roty.mulBefore(rotx));
			}
			else {
				out.toRotationMatrix(rotout);
			}
			Matrix4 everything = new Matrix4(rotout.mulBefore(scaleout));
			
			everything.set(0, 3, transout.x);
			everything.set(1, 3, transout.y);
			everything.set(2, 3, transout.z);
			everything.set(3, 3, 1);
			object.transformation.set(everything);
			
		}
	
	else {
		//handles edge cases
			AnimKeyframe val = next !=null ? next : prev;
			
			object.transformation.set(val.transformation);
		}
	}
}

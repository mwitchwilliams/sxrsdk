/* Copyright 2015 Samsung Electronics Co., LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.samsungxr.x3d.ar;

import android.view.MotionEvent;

import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRContext;
import com.samsungxr.SXREventListeners;
import com.samsungxr.SXRLight;
import com.samsungxr.SXRPicker;
import com.samsungxr.SXRPointLight;
import com.samsungxr.SXRScene;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRShaderId;
import com.samsungxr.SXRTransform;
import com.samsungxr.mixedreality.IMixedReality;
import com.samsungxr.mixedreality.SXRMixedReality;
import com.samsungxr.ITouchEvents;
import com.samsungxr.mixedreality.SXRAnchor;
import com.samsungxr.mixedreality.SXRHitResult;

import com.samsungxr.mixedreality.SXRPlane;
import com.samsungxr.mixedreality.SXRTrackingState;
import com.samsungxr.mixedreality.IAnchorEvents;
import com.samsungxr.mixedreality.IPlaneEvents;
import com.samsungxr.mixedreality.arcore.ARCoreAnchor;
import com.samsungxr.x3d.AnimationInteractivityManager;
import com.samsungxr.x3d.ShaderSettings;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.samsungxr.utility.Log;


public class ARMain {
    private static String TAG = "ARMain";
    //private static int MAX_VIRTUAL_OBJECTS = 20;
    private static int MAX_VIRTUAL_OBJECTS = 8;

    private SXRContext mSXRContext;
    private SXRScene mMainScene;
    private SXRMixedReality mMixedReality;
    private ARHelper helper;
    private DragHandler mTouchHandler;

    private List<SXRAnchor> mVirtualObjects;
    private int mVirtObjCount = 0;
    private ShaderSettings mShaderSettings = null;
    private SXRShaderId mX3DShader;

    private SelectionHandler mSelector;

    private String mFilename;

    private boolean mInitialPlane = true;
    private boolean mInitAnchorNodeSet = false;
    private SXRNode mInitAnchorNode = null;
    private SXRNode mRoot = null;
    private AnimationInteractivityManager mAnimationInteractivityManager;

    public ARMain(SXRContext sxrContext, SXRNode root,
                  ShaderSettings shaderSettings, SXRShaderId x3DShader,
                  AnimationInteractivityManager animationInteractivityManager) {
        Log.e("X3DDBG", "ARMain constructor BGN.");

        mSXRContext = sxrContext;
        mMainScene = mSXRContext.getMainScene();
        helper = new ARHelper();
        mTouchHandler = new DragHandler();
        mVirtualObjects = new ArrayList<>() ;
        mVirtObjCount = 0;
        mShaderSettings = shaderSettings;
        mX3DShader = x3DShader;
        mRoot = root;
        mAnimationInteractivityManager = animationInteractivityManager;

        mMixedReality = new SXRMixedReality(mMainScene);
        mMixedReality.getEventReceiver().addListener(planeEventsListener);
        mMixedReality.getEventReceiver().addListener(anchorEventsListener);

    }

    public void setInitAnchorNode(boolean initAnchorNodeSet, SXRNode initAnchorNode) {
        mInitAnchorNodeSet = initAnchorNodeSet;
        mInitAnchorNode = initAnchorNode;
    }

    public void resume() {

        mMixedReality.resume();
        Log.e("X3DDBG", "ARMain resume() END.");
    }

/*
    @Override
    public void onStep() {
        super.onStep();
        for (GVRAnchor anchor: mVirtualObjects) {
            for (GVRSceneObject obj: anchor.getChildren()) {
                ((VirtualObject)obj).reactToLightEnvironment(
                        mixedReality.getLightEstimate().getPixelIntensity());
            }
        }
    }
*/

    /**
     * The plane events listener handles plane detection events.
     * It also handles initialization and shutdown.
     */
    private IPlaneEvents planeEventsListener = new IPlaneEvents() {
        /**
         * Get the depth of the touch screen in the 3D world
         * and give it to the cursor controller so touch
         * events will be handled properly.
         */
        @Override
        public void onStartPlaneDetection(IMixedReality mr) {
            //gvrPlane.setSceneObject(helper.createQuadPlane(getGVRContext()));
            Log.e("X3DDBG", "ARMain onStartPlaneDetection.");
            if (mInitialPlane) {
                Log.e("X3DDBG", "  onStartPlaneDetection INITIAL PLANE.");
            }
            //sxrPlane.setNode(helper.createQuadPlane(mSXRContext));
            //mainScene.addNode(sxrPlane);
            float screenDepth = mr.getScreenDepth();
            mr.getPassThroughObject().getEventReceiver().addListener(mTouchHandler);
            helper.initCursorController(mSXRContext, mTouchHandler, screenDepth);
        }

        @Override
        public void onStopPlaneDetection(IMixedReality mr) {

        }

        /**
         * Place a transparent quad in the 3D scene to indicate
         * vertically upward planes (floor, table top).
         * We don't need colliders on these since they are
         * not pickable.
         */
        @Override
        public void onPlaneDetected(SXRPlane plane) {
            Log.e("X3DDBG", "ARMain onPlaneDetected.");
            if (mInitialPlane) {
                Log.e("X3DDBG", "ARMain onPlaneDetected INITIAL PLANE.");
            }
            //mInitialPlane = false;
            if (plane.getPlaneType() == SXRPlane.Type.VERTICAL)
            {
                return;
            }
            SXRNode planeMesh = helper.createQuadPlane(mSXRContext);

            float[] pose = new float[16];

            plane.getCenterPose(pose);
            Log.e("X3DDBG", "ARMain onPlaneDetected createQuad: " + planeMesh.getName());
            /*
            Log.e("X3DDBG", "      pose[][]= [" + pose[ 0] + ", " + pose[ 4] + ", " + pose[ 8] + ", " + pose[12] + "]");
            Log.e("X3DDBG", "                [" + pose[ 1] + ", " + pose[ 5] + ", " + pose[ 9] + ", " + pose[13] + "]");
            Log.e("X3DDBG", "                [" + pose[ 2] + ", " + pose[ 6] + ", " + pose[10] + ", " + pose[14] + "]");
            Log.e("X3DDBG", "                [" + pose[ 3] + ", " + pose[ 7] + ", " + pose[11] + ", " + pose[15] + "]");
*/
            Log.e("X3DDBG", "      pose[12][13][14]= (" + pose[12] + ", " + pose[13] + ", " + pose[14] + ")");

            planeMesh.attachComponent(plane);
            //Log.e("X3DDBG", "planeMesh(x, y, z): ( " + planeMesh.getTransform().getPositionX()
            //    + ", " + planeMesh.getTransform().getPositionY() + ", " + planeMesh.getTransform().getPositionZ() + ")");
            mMainScene.addNode(planeMesh);
            //addVirtualObject(pose);
        }  //  end onPlaneDetected

        /**
         * Show/hide the 3D plane node based on whether it
         * is being tracked or not.
         */

        @Override
        public void onPlaneStateChange(SXRPlane sxrPlane, SXRTrackingState trackingState) {
            Log.e("X3DDBG", "ARMain onPlaneStateChange trackingState=" + trackingState);
            sxrPlane.setEnable(trackingState == SXRTrackingState.TRACKING);
            if (mInitialPlane) {
                SXRMixedReality sxrMixedReality = getSXRMixedReality();
                Log.e("X3DDBG", "ARMain onPlaneStateChange INITIAL PLANE mInitialPlane=" + mInitialPlane);
                float[] pose = {1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1};
                SXRNode arAnchorObj = null;
                try {
                    arAnchorObj = sxrMixedReality.createAnchorNode(pose);
                    Log.e("X3DDBG", "onPlaneStateChange: arAnchorObj = sxrMixedReality.createAnchorNode(pose) OK");
                }
                catch (com.google.ar.core.exceptions.NotTrackingException nte) {
                    Log.e("X3DDBG", "onPlaneStateChange arAnchorObj NotTrackingException " + nte);
                }
                catch (Exception e) {
                    Log.e("X3DDBG", "onPlaneStateChange arAnchorObj Exception " + e);
                }
                ARCoreAnchor anchor = (ARCoreAnchor) arAnchorObj.getComponent(SXRAnchor.getComponentType());
                // commented out cause I reset 'setTrackingState' back to protected
                //anchor.setTrackingState(SXRTrackingState.PAUSED );
                //SXRAnchor sxrAnchor = (SXRAnchor) anchor;
                Log.e("X3DDBG", "   onPlaneStateChange addSXRAnchor()");
                addSXRAnchor( (SXRAnchor) anchor );
                Log.e("X3DDBG", "   onPlaneStateChange returned from addSXRAnchor()");
                mRoot.addChildObject( arAnchorObj );

            }
            mInitialPlane = false;

        }

        @Override
        public void onPlaneMerging(SXRPlane childPlane, SXRPlane parentPlane) {
            //Log.e("X3DDBG", "ARMain onPlaneMerging parent(" + parentPlane.getTransform().getPositionX()
            //+ ", " + parentPlane.getTransform().getPositionY()+ ", " + parentPlane.getTransform().getPositionZ());

        }
    };  // end IPlaneEvents
    /*{
        @Override
        public void onPlaneDetection(SXRPlane sxrPlane) {
            //gvrPlane.setSceneObject(helper.createQuadPlane(getGVRContext()));
            Log.e("X3DDBG", "ARMain onPlaneDetection.");
            sxrPlane.setNode(helper.createQuadPlane(mSXRContext));
            mainScene.addNode(sxrPlane);
        }

        @Override
        public void onPlaneStateChange(SXRPlane sxrPlane, SXRTrackingState gvrTrackingState) {
            if (gvrTrackingState != SXRTrackingState.TRACKING) {
                sxrPlane.setEnable(false);
                Log.e("X3DDBG", "ARMain onPlaneStateChange: gvrPlane.setEnable(false).");
            }
            else {
                sxrPlane.setEnable(true);
                Log.e("X3DDBG", "ARMain onPlaneStateChange: gvrPlane.setEnable(true).");
            }
        }

        @Override
        public void onPlaneMerging(SXRPlane gvrPlane, SXRPlane gvrPlane1) {
        }
    };
*/
    /**
     * Show/hide the 3D node associated with the anchor
     * based on whether it is being tracked or not.
     */
    private IAnchorEvents anchorEventsListener = new IAnchorEvents() {
        @Override
        public void onAnchorStateChange(SXRAnchor SXRAnchor, SXRTrackingState state)
        {
            SXRAnchor.setEnable(state == SXRTrackingState.TRACKING);
        }
    };



    /**
     * Handles selection hilighting, rotation and scaling
     * of currently selected 3D object.
     * A light attached to the parent of the
     * selected 3D object is used for hiliting it.
     * The root of the hierarchy can be rotated or scaled.
     */
    static public class SelectionHandler implements ITouchEvents {
        static final int DRAG = 1;
        static final int SCALE_ROTATE = -1;
        static final int UNTOUCHED = 0;
        static private SXRNode mSelected = null;
        private int mSelectionMode = UNTOUCHED;
        private final float[] PICKED_COLOR = {0.4f, 0.6f, 0, 1.0f};
        private final float[] UPDATE_COLOR = {0.6f, 0, 0.4f, 1.0f};
        private final float[] DRAG_COLOR = {0, 0.6f, 0.4f, 1.0f};
        private SXRNode mSelectionLight;
        private IMixedReality mMixedReality;
        private float mHitY;
        private float mHitX;

        public SelectionHandler(SXRContext ctx, IMixedReality mr) {
            super();
            mMixedReality = mr;
            mSelectionLight = new SXRNode(ctx);
            mSelectionLight.setName("SelectionLight");
            SXRPointLight light = new SXRPointLight(ctx);
            light.setSpecularIntensity(0.1f, 0.1f, 0.1f, 0.1f);
            mSelectionLight.attachComponent(light);
            mSelectionLight.getTransform().setPositionZ(1.0f);
        }

        public static SXRNode getSelected() {
            return mSelected;
        }

        /*
         * When entering an anchored object, it is hilited by
         * adding a point light under its parent.
         */
        public void onEnter(SXRNode target, SXRPicker.SXRPickedObject pickInfo) {
            Log.e("X3DDBG", "ARMain SelectionHandler onEnter.");
            if (mSelected != null) {
                return;
            }
            SXRPointLight light =
                    (SXRPointLight) mSelectionLight.getComponent(SXRLight.getComponentType());
            light.setDiffuseIntensity(PICKED_COLOR[0],
                    PICKED_COLOR[1],
                    PICKED_COLOR[1],
                    PICKED_COLOR[2]);
            SXRNode lightParent = mSelectionLight.getParent();
            SXRNode targetParent = target.getParent();

            if (lightParent != null) {
                if (lightParent != targetParent) {
                    lightParent.removeChildObject(mSelectionLight);
                    targetParent.addChildObject(mSelectionLight);
                    mSelectionLight.getComponent(SXRLight.getComponentType()).enable();
                } else {
                    mSelectionLight.getComponent(SXRLight.getComponentType()).enable();
                }
            } else {
                targetParent.addChildObject(mSelectionLight);
                mSelectionLight.getComponent(SXRLight.getComponentType()).enable();
            }
        }

        /*
         * When the object is no longer selected, its selection light is disabled.
         */
        public void onExit(SXRNode sceneObj, SXRPicker.SXRPickedObject pickInfo) {
            Log.e("X3DDBG", "ARMain SelectionHandler onExit.");
            if ((mSelected == sceneObj) || (mSelected == null)) {
                mSelectionLight.getComponent(SXRLight.getComponentType()).disable();
                mSelected = null;
            }
        }

        /*
         * The color of the selection light changes when the object is being dragged.
         * If another object is already selected, ignore the touch event.
         */
        public void onTouchStart(SXRNode sceneObj, SXRPicker.SXRPickedObject pickInfo) {
            Log.e("X3DDBG", "ARMain SelectionHandler onTouchStart BGN");
            if (pickInfo.motionEvent == null) {
                return;
            }
            if (mSelected == null) {
                startTouch(sceneObj,
                        pickInfo.motionEvent.getX(),
                        pickInfo.motionEvent.getY(),
                        SCALE_ROTATE);
            }
        }

        public void onTouchEnd(SXRNode sceneObj, SXRPicker.SXRPickedObject pickInfo) {
        }

        int cnt = 0;
        public void onInside(SXRNode sceneObj, SXRPicker.SXRPickedObject pickInfo) {
            if ( (cnt % 1000) == 0) {
                Log.e("X3DDBG", "ARMain SelectionHandler TouchHandler onInside(), cnt=" + (cnt / 1000));
            }
            cnt++;
        }

        public void onMotionOutside(SXRPicker picker, MotionEvent event) {
        }

        /*
         * Rotate and scale the object relative to its current state.
         * The node being rotated / scaled is a child
         * of the anchored object (which is being oriented and positioned
         * by MixedReality).
         */
        private void scaleRotate(float rotateDelta, float scaleDelta) {
            Log.e("X3DDBG", "ARMain SelectionHandler scaleRotate( , ) BGN");
            SXRNode selected = getSelected();
            SXRTransform t = selected.getTransform();
            float scale = t.getScaleX();
            Quaternionf q = new Quaternionf();
            Vector3f ea = new Vector3f();
            float angle = rotateDelta / 10.0f;

                /*
                 * rotate about Y axis
                 */
            q.set(t.getRotationX(), t.getRotationY(), t.getRotationZ(), t.getRotationW());
            q.getEulerAnglesXYZ(ea);
            q.rotateAxis(angle, 0, 1, 0);

                /*
                 * scale the model
                 */
            scale += scaleDelta / 20.0f;
            if (scale < 0.1f) {
                scale = 0.1f;
            } else if (scale > 50.0f) {
                scale = 50.0f;
            }
            t.setRotation(q.w, q.x, q.y, q.z);
            t.setScale(scale, scale, scale);
        }

        private void drag(float x, float y) {
            SXRAnchor anchor = (SXRAnchor) mSelected.getParent().getComponent(SXRAnchor.getComponentType());

            if (anchor != null) {
                SXRHitResult hit = mMixedReality.hitTest(x, y);

                if (hit != null) {                           // move the object to a new position
                    mMixedReality.updateAnchorPose(anchor, hit.getPose());
                }
            }
        }

        public void update(SXRPicker.SXRPickedObject pickInfo) {
            float x = pickInfo.motionEvent.getX();
            float y = pickInfo.motionEvent.getY();

            if (mSelectionMode == SCALE_ROTATE) {
                float dx = (x - mHitX) / 100.0f;
                float dy = (y - mHitY) / 100.0f;
                scaleRotate(dx, dy);
            } else if (mSelectionMode == DRAG) {
                drag(x, y);
            }
        }

        public void startTouch(SXRNode sceneObj, float hitx, float hity, int mode) {
            Log.e("X3DDBG", "ARMain SelectionHandler startTouch( , , , ) BGN");
            SXRPointLight light =
                    (SXRPointLight) mSelectionLight.getComponent(SXRLight.getComponentType());
            mSelectionMode = mode;
            mSelected = sceneObj;
            if (mode == DRAG) {
                light.setDiffuseIntensity(DRAG_COLOR[0],
                        DRAG_COLOR[1],
                        DRAG_COLOR[1],
                        DRAG_COLOR[2]);
            } else {
                light.setDiffuseIntensity(UPDATE_COLOR[0],
                        UPDATE_COLOR[1],
                        UPDATE_COLOR[1],
                        UPDATE_COLOR[2]);
            }
            mHitX = hitx;
            mHitY = hity;
        }

        public void endTouch() {
            SXRPointLight light =
                    (SXRPointLight) mSelectionLight.getComponent(SXRLight.getComponentType());
            light.setDiffuseIntensity(PICKED_COLOR[0],
                    PICKED_COLOR[1],
                    PICKED_COLOR[1],
                    PICKED_COLOR[2]);
            mSelected = null;
            mSelectionMode = UNTOUCHED;
        }
    }


    /**
     * Handles touch events for the screen
     * (those not inside 3D anchored objects).
     * If phone AR is being used with passthru video,
     * the object displaying the camera output also
     * has a collider and is touchable.
     * This is how picking is handled when using
     * the touch screen.
     *
     * Tapping the screen or clicking on a plane
     * will cause a 3D object to be placed there.
     * Dragging with the controller or your finger
     * inside the object will scale it (Y direction)
     * and rotate it (X direction). Dragging outside
     * a 3D object will drag the currently selected
     * object (the last one you added/manipulated).
     */
    public class DragHandler extends SXREventListeners.TouchEvents {

        @Override
        public void onTouchStart(SXRNode sceneObj, SXRPicker.SXRPickedObject pickInfo) {
            //Log.e("X3DDBG", "ARMain DragHandler onTouchStart()");
        }

        @Override
        public void onTouchEnd(SXRNode sceneObj, SXRPicker.SXRPickedObject pickInfo) {
            Log.e("X3DDBG", "ARMain DragHandler onTouchEnd( , )");
            if (SelectionHandler.getSelected() != null) {
                Log.e("X3DDBG", "ARMain onTouchEnd() SelectionHandler.getSelected() != null");
                mSelector.endTouch();
            } else {
                //Log.e("X3DDBG", "ARMain onTouchEnd() findAnchorNear(" + pickInfo.hitLocation[0] + ", " +
                //                                pickInfo.hitLocation[1] + ", " +
                //                                pickInfo.hitLocation[2] +")");
                SXRAnchor anchor = findAnchorNear(pickInfo.hitLocation[0],
                        pickInfo.hitLocation[1],
                        pickInfo.hitLocation[2],
                        300);
            //    if (anchor != null) {
            //        Log.e("X3DDBG", "ARMain onTouchEnd() anchor != null");
            //        return;
            //    }
                Log.e("X3DDBG", "ARMain onTouchEnd() anchor(x,y,z) (" + anchor.getTransform().getPositionX()
                        + ", " + anchor.getTransform().getPositionY() + ", " + anchor.getTransform().getPositionZ() +")" );
                float x = pickInfo.motionEvent.getX();
                float y = pickInfo.motionEvent.getY();
                SXRHitResult hit = mMixedReality.hitTest(x, y);
                if (hit != null) {
                    Log.e("X3DDBG", "ARMain onTouchEnd() hit != null, (x, y) " + x + ", " + y + " Call addVirtualObject()");
                    addVirtualObject(hit.getPose());
                }
                else Log.e("X3DDBG", "ARMain onTouchEnd() hit == null");
            }
        }

        public void onInside(SXRNode sceneObj, SXRPicker.SXRPickedObject pickInfo) {
            //Log.e("X3DDBG", "ARMain DragHandler onInside( , )");
            SXRNode selected = mSelector.getSelected();

            if (pickInfo.motionEvent == null) {
                return;
            }
            if (pickInfo.touched)           // currently touching an object?
            {
                if (selected != null)       // is a 3D object selected?
                {
                    mSelector.update(pickInfo);
                } else {
                    SXRAnchor anchor = findAnchorNear(pickInfo.hitLocation[0],
                            pickInfo.hitLocation[1],
                            pickInfo.hitLocation[2],
                            150);
                    if (anchor != null) {
                        selected = anchor.getOwnerObject();
                        mSelector.startTouch(selected.getChildByIndex(0),
                                pickInfo.motionEvent.getX(),
                                pickInfo.motionEvent.getY(),
                                SelectionHandler.DRAG);
                    }
                }
            }
        }

        /**
         * Look for a 3D object in the scene near the given position.
         * Used ro prevent objects from being placed too close together.
         */
        private SXRAnchor findAnchorNear(float x, float y, float z, float maxdist)
        {
            //Log.e("X3DDBG", "ARMain findAnchorNear(" + x + ", " + y + ", " + z + ")");
            Matrix4f anchorMtx = new Matrix4f();
            Vector3f v = new Vector3f();
            for (SXRAnchor anchor : mVirtualObjects)
            {
                //Log.e("X3DDBG", "ARMain findAnchorNear for anchor");
                float[] anchorPose = anchor.getPose();
                anchorMtx.set(anchorPose);
                anchorMtx.getTranslation(v);
                v.x -= x;
                v.y -= y;
                v.z -= z;
                float d = v.length();
                if (d < maxdist)
                {
                    //Log.e("X3DDBG", "ARMain findAnchorNear return anchor");
                    return anchor;
                }
            }
            //Log.e("X3DDBG", "ARMain findAnchorNear return null");
            return null;
        }
    };


    /**
     * The file that will be the AR objects
     * @param filename
     */
    public void setX3DFile(String filename ) {
        mFilename = filename;
    }
    /**
     * Loads a 3D model using the asset loaqder and attaches
     * a collider to it so it can be picked.
     * If you are using phone AR, the touch screen can
     * be used to drag, rotate or scale the object.
     * If you are using a headset, the controller
     * is used for picking and moving.
     */
    private SXRNode load3dModel(final SXRContext sxrContext) throws IOException
    {
        //final SXRNode sceneObject = sxrContext.getAssetLoader().loadModel("objects/andy.obj");
        //final SXRNode sceneObject = sxrContext.getAssetLoader().loadModel("conered.x3d");
        //final SXRNode sceneObject = sxrContext.getAssetLoader().loadModel("animation01.x3d");
        //final SXRNode sceneObject = sxrContext.getAssetLoader().loadModel("teapotandtorus.x3d");
        //final SXRNode sxrNode = sxrContext.getAssetLoader().loadModel("RGBConesAndCylinder.x3d");
        final SXRNode sxrNode = sxrContext.getAssetLoader().loadModel( mFilename );
        //sxrNode.attachComponent(new SXRBoxCollider(sxrContext));
        //sxrNode.getEventReceiver().addListener(mSelector);
        attachComponentsAndEvents(sxrContext, sxrNode);
        return sxrNode;
    }

    public void attachComponentsAndEvents(final SXRContext sxrContext, final SXRNode sxrNode)
    {
        if (sxrNode != null) {
            //final SXRNode sxrNode = sxrContext.getAssetLoader().loadModel("RGBConesAndCylinder.x3d");
            sxrNode.attachComponent(new SXRBoxCollider(sxrContext));
            sxrNode.getEventReceiver().addListener(mSelector);
            //return sxrNode;
        }
    }

    public void addSXRAnchor( SXRAnchor sxrAnchor) {
        mVirtualObjects.add( sxrAnchor );
        mVirtObjCount++;
    }

    private void addVirtualObject(float[] pose) {

        if (mVirtObjCount >= MAX_VIRTUAL_OBJECTS)
        {
            Log.e("X3DDBG", "ARMain addVirtualObject() MAXXED OUT: mVirtObjCount >= MAX_VIRTUAL_OBJECTS");
            return;
        }
        try
        {
            Log.e("X3DDBG", "ARMain addVirtualObject() mVirtObjCount=" + mVirtObjCount);
            Log.e("X3DDBG", "      pose[][]= [" + pose[ 0] + ", " + pose[ 4] + ", " + pose[ 8] + ", " + pose[12] + "]");
            Log.e("X3DDBG", "                [" + pose[ 1] + ", " + pose[ 5] + ", " + pose[ 9] + ", " + pose[13] + "]");
            Log.e("X3DDBG", "                [" + pose[ 2] + ", " + pose[ 6] + ", " + pose[10] + ", " + pose[14] + "]");
            Log.e("X3DDBG", "                [" + pose[ 3] + ", " + pose[ 7] + ", " + pose[11] + ", " + pose[15] + "]");

            SXRNode arModel = null;
            if (mInitAnchorNodeSet) {
                Log.e("X3DDBG", "ARMain addVirtualObject, add initial scene");
                arModel = mInitAnchorNode;
                //mInitAnchorNodeSet = false;
                /*
                if (mAnimationInteractivityManager != null) {
                    try {
                        mAnimationInteractivityManager.initAnimationsAndInteractivity();
                        // Need to build a JavaScript function that constructs the
                        // X3D data type objects used with a SCRIPT.
                        // Scripts can also have an initialize() method.
                        mAnimationInteractivityManager.InitializeScript();
                    } catch (Exception exception) {
                        Log.e(TAG, "Error initialing X3D <ROUTE> or <Script> node related to Animation or Interactivity.");
                    }
                }
                */
            }
            else {
                Log.e("X3DDBG", "ARMain addVirtualObject, add INLINE scene");
                //SXRNode arModel = load3dModel(mSXRContext);
                arModel = load3dModel(mSXRContext);
            }
            //anchor = mixedReality.createAnchor(pose, andy);
            SXRNode anchorObj = mMixedReality.createAnchorNode(pose);

            anchorObj.addChildObject(arModel);

            SXRAnchor anchor = (SXRAnchor) anchorObj.getComponent(SXRAnchor.getComponentType());
            addSXRAnchor( anchor );
            mMainScene.addNode(anchorObj);

            if (mInitAnchorNodeSet) {
                // initial scene, now set up the animations and interactivity
                Log.e("X3DDBG", "ARMain addVirtualObject, add initial scene");
                if (mAnimationInteractivityManager != null) {
                    try {
                        SXRNode sxrNodeCapture = mMainScene.getNodeByName("OrangeCone");
                        if ( sxrNodeCapture != null ) Log.e("X3DDBG", "ARMain Augmented Reality found OrangeCone.");
                        else Log.e("X3DDBG", "ARMain Augmented Reality NOT fine OrangeCone.");
                        //if ( mMainScene == mRoot) Log.e("X3DDBG", "ARMain Augmented Reality <ROUTE> / <Script> Animation / Interactivity 0.");
                        //Log.e("X3DDBG", "ARMain Augmented Reality <ROUTE> / <Script> Animation / Interactivity 0.");
                        //mRoot.addChildObject( mInitAnchorNode );
                        Log.e("X3DDBG", "ARMain Augmented Reality <ROUTE> / <Script> Animation / Interactivity 1.");
                        mAnimationInteractivityManager.initAnimationsAndInteractivity();
                        Log.e("X3DDBG", "ARMain Augmented Reality <ROUTE> / <Script> Animation / Interactivity 2.");
                        // Need to build a JavaScript function that constructs the
                        // X3D data type objects used with a SCRIPT.
                        // Scripts can also have an initialize() method.
                        mAnimationInteractivityManager.InitializeScript();
                        Log.e("X3DDBG", "ARMain Augmented Reality <ROUTE> / <Script> Animation / Interactivity 3.");
                    } catch (Exception exception) {
                        Log.e("X3DDBG", "Error initialing X3D Augmented Reality <ROUTE> or <Script> Animation or Interactivity: " + exception);
                        Log.e(TAG, "Error initialing X3D Augmented Reality <ROUTE> or <Script> Animation or Interactivity.");
                    }
                }
                mInitAnchorNodeSet = false;
            }

        }
        catch (IOException ex)
        {
            ex.printStackTrace();
            Log.e(TAG, ex.getMessage());
        }
    }

    public SXRMixedReality getSXRMixedReality() {
        Log.e("X3DDBG", "ARMain getSXRMixedReality()");
        return mMixedReality;
    }

}
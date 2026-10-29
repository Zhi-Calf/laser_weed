"""Ground-plane PnP and the same explicit rigid-body ground model as the C++ SDK."""
import math
import cv2
import numpy as np


def orientation(roll, pitch):
    cr,sr,cp,sp = math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch)
    return np.array([[cp,sp*sr,sp*cr],[0,cr,-sr],[-sp,cp*sr,cp*cr]])


def body_points(xy, sync):
    r = orientation(sync['reference_roll_rad'],sync['reference_pitch_rad'])
    return (np.c_[xy, np.zeros(len(xy))]-[0,0,sync['origin_height_m']]) @ r


def homography(k, rotation, position, height, roll, pitch):
    n = orientation(roll,pitch)[2,:]
    plane = np.array([[1,0,-position[0]],[0,1,-position[1]],
        [-n[0]/n[2],-n[1]/n[2],-height/n[2]-position[2]]])
    ground_to_bev = np.array([[0,-100,320],[-100,0,799],[0,0,1.]])
    return ground_to_bev @ np.linalg.inv(k @ rotation.T @ plane)


def image_to_body(h, uv):
    q = np.c_[uv, np.ones(len(uv))] @ h.T
    if np.any(abs(q[:,2]) < 1e-9):
        raise ValueError('投影接近地平线')
    q = q[:,:2]/q[:,2:]
    return np.c_[(799-q[:,1])*.01, (320-q[:,0])*.01]


def solve(k, uv, xy, fit, inliers, sync, measured_h):
    # Solve in the measured level-ground frame first; transform to rigid vehicle axes.
    points = np.c_[xy,np.zeros(len(xy))].astype(np.float64)
    selected = np.flatnonzero(fit)[np.asarray(inliers).ravel().astype(bool)]
    if len(selected) < 6:
        raise ValueError('动态外参至少需要 6 个可靠标定内点')
    found, rvecs, tvecs, _ = cv2.solvePnPGeneric(points[selected],uv[selected],k,None,flags=cv2.SOLVEPNP_IPPE)
    candidates=[]
    for rv,tv in zip(rvecs,tvecs):
        rv,tv=cv2.solvePnPRefineLM(points[selected],uv[selected],k,None,rv,tv)
        rc=cv2.Rodrigues(rv)[0]; tc=tv.reshape(3)
        camera_ground=-rc.T@tc
        if not .1 < camera_ground[2] < 10 or np.any((points@rc.T+tc)[:,2] <= .05):
            continue
        predicted=cv2.projectPoints(points[selected],rv,tv,k,None)[0].reshape(-1,2)
        rmse=float(np.sqrt(np.mean(np.sum((predicted-uv[selected])**2,axis=1))))
        candidates.append((rmse,rc,tc,camera_ground))
    if not found or not candidates:
        raise ValueError('无法得到相机位于地面上方、点位于相机前方的外参')
    candidates.sort(key=lambda x:x[0]); rms,rc,tc,camera_ground=candidates[0]
    if len(candidates)>1 and abs(candidates[1][0]-rms)<.1 and np.linalg.norm(candidates[1][3]-camera_ground)>.05:
        raise ValueError('平面外参存在歧义，请增加覆盖范围并重新标定')
    if rms>3:
        raise ValueError('外参重投影误差超过 3 像素，请检查测量和点选')
    rg=orientation(sync['reference_roll_rad'],sync['reference_pitch_rad'])
    rotation=rg.T@rc.T
    position=rg.T@(camera_ground-[0,0,sync['origin_height_m']])
    h=homography(k,rotation,position,sync['origin_height_m'],sync['reference_roll_rad'],sync['reference_pitch_rad'])
    actual=image_to_body(h,uv)
    expected=body_points(xy,sync)[:,:2]
    errors=np.linalg.norm(actual-expected,axis=1)
    check=errors[~fit]
    if np.sqrt(np.mean(check**2))>.05 or check.max()>.1:
        raise ValueError('物理外参的独立地面检查未通过')
    # Match the exact SDK reference grid to avoid publishing a model it will reject.
    grid=np.array([[x,y] for x in (1.,3.,5.) for y in (-.5,0.,.5)])
    pixels=np.c_[320-grid[:,1]*100,799-grid[:,0]*100,np.ones(len(grid))]@np.linalg.inv(h).T
    pixels=pixels[:,:2]/pixels[:,2:]
    ref_error=float(np.linalg.norm(image_to_body(measured_h,pixels)-grid,axis=1).max())
    if ref_error>.05:
        raise ValueError('动态模型与实测 BEV 在运行参考范围内偏差超过 5 cm')
    result=dict(camera_to_vehicle_rotation=rotation.tolist(),camera_position_vehicle_m=position.tolist(),
        origin_height_m=sync['origin_height_m'],reference_roll_rad=sync['reference_roll_rad'],
        reference_pitch_rad=sync['reference_pitch_rad'],camera_height_ground_m=float(camera_ground[2]),
        camera_down_angle_deg=math.degrees(math.atan2(-rotation[2,2],math.hypot(rotation[0,2],rotation[1,2]))),
        camera_left_angle_deg=math.degrees(math.atan2(rotation[1,2],rotation[0,2])),
        reprojection_rmse_px=rms,check_rmse_m=float(np.sqrt(np.mean(check**2))),check_max_m=float(check.max()),
        max_reference_error_m=ref_error,validated=True)
    return result


def validate_file(bev_path, intrinsic_path):
    fs=cv2.FileStorage(str(bev_path),cv2.FILE_STORAGE_READ)
    node=fs.getNode('dynamic_bev')
    if node.empty():
        fs.release(); return False
    if node.getNode('enabled').real()!=1:
        raise ValueError('动态发布必须明确启用物理模型')
    r=node.getNode('camera_to_vehicle_rotation').mat()
    t=node.getNode('camera_position_vehicle_m').mat()
    keys=('origin_height_m','reference_roll_rad','reference_pitch_rad')
    if any(node.getNode(key).empty() for key in keys):
        raise ValueError('缺少动态外参字段')
    height,roll,pitch=(node.getNode(key).real() for key in keys)
    measured=fs.getNode('image_to_bev').mat();fs.release()
    intrinsic=cv2.FileStorage(str(intrinsic_path),cv2.FILE_STORAGE_READ)
    k=intrinsic.getNode('camera_matrix').mat();intrinsic.release()
    if (r is None or t is None or r.shape!=(3,3) or t.shape!=(3,1) or
        not np.isfinite(r).all() or not np.isfinite(t).all() or
        np.linalg.norm(r.T@r-np.eye(3))>1e-6 or abs(np.linalg.det(r)-1)>1e-6 or
        not .05<=height<=3 or not all(math.isfinite(v) and abs(v)<=math.radians(5) for v in (roll,pitch))):
        raise ValueError('动态外参矩阵或高度无效')
    if orientation(roll,pitch)[2]@t.ravel()+height<=.1:
        raise ValueError('相机必须在地面上方')
    h=homography(k,r,t.ravel(),height,roll,pitch)
    ground=np.array([[x,y] for x in (1.,3.,5.) for y in (-.5,0.,.5)])
    q=np.c_[320-ground[:,1]*100,799-ground[:,0]*100,np.ones(len(ground))]@np.linalg.inv(h).T
    if not np.isfinite(q).all() or np.any(abs(q[:,2])<1e-9):
        raise ValueError('动态参考投影无效')
    if np.linalg.norm(image_to_body(measured,q[:,:2]/q[:,2:])-ground,axis=1).max()>.05:
        raise ValueError('动态参考投影与实测 BEV 不一致')
    return True

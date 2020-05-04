import numpy as np
import cv2 

class RANSAC():
    """Class for performing ransac outlier rejection """
    def __init__(self):
        #initiate orb features and brute force matcher
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        #set max iterations
        self.max_iter = 1000

        #Ransac threshold for keeping inliers:
        self.thres = 50

    def calc_scale_angle(self, anchor, arm, kpq, kp):
        """Function that calculates the scale and angle required to 
        transform the query image to reference image
        Args:
            anchor: first match between features of the 2 images
            arm: second match between features of the 2 images
            Kpq: keypoints of the first image
            Kp: keypoints of the second image
        Returns:
            scale: stretching factor for transforming image 1 to 2
            angle: rotation for transforming image 1 to 2
        """

        #anchor coordinates 
        idx_q = anchor.queryIdx
        idx_t = anchor.trainIdx

        (yq1, xq1) = kpq[idx_q].pt
        (yt1, xt1) = kp[idx_t].pt


        #arm coordinates
        idx_q = arm.queryIdx
        idx_t = arm.trainIdx

        (yq2, xq2) = kpq[idx_q].pt
        (yt2, xt2) = kp[idx_t].pt

        #Form 2 vectors
        v_q = np.array([xq2-xq1, yq2-yq1, 0])
        v_t = np.array([xt2-xt1, yt2-yt1, 0])

        scale = np.linalg.norm(v_t) / np.linalg.norm(v_q)

        #normalize vectors
        v_q = v_q / np.linalg.norm(v_q)
        v_t = v_t / np.linalg.norm(v_t)

        #Calculate the angle
        angq= np.arctan2(v_q[1],v_q[0])
        angt= np.arctan2(v_t[1],v_t[0])
        diff = angt - angq

        if diff > np.pi:
            angle = diff - 2*np.pi
        elif diff < -np.pi:
            angle = diff + 2*np.pi
        else: 
            angle = diff

        return scale, angle


    def transform(self, x, y,x_anch, y_anch, scale, angle):
        """Function that transforms the features from one image to the 
        other using scale and angle
        Args:
            x, y: coordinates of point to be transformed
            x_anch, y_anch: coordinates of anchor point to perform rotation
            scale: stretching factor
            angle: rotation amount
        Returns:
            v_new: vector containing the transformed coordinates
        """

        rot = np.array([[np.cos(angle), -np.sin(angle)],
                        [np.sin(angle), np.cos(angle)]])
        v = np.array([x-x_anch, y-y_anch]).reshape(-1,1)

        v_new = scale * rot.dot(v)

        return v_new[0,0], v_new[1,0]

    def ransac(self, imgq, imgt):
        """Function that performs RANSAC outlier rejection for matched 
        features between 2 images
        Args:
            imgq: query image
            imgt: reference image
        Returns:
            best_count: best inlier count of matches
            avg_angle: average rotation angle from imgq to imgt
        """

        #detect features
        kpq, desq = self.orb.detectAndCompute(imgq,None)
        kpt, dest = self.orb.detectAndCompute(imgt,None)

        #compute matches
        matches = self.bf.match(desq, dest)

        max_count = 0

        #RANSAC algorithm
        for it in range(self.max_iter):

            #select 2 random matches
            [anchor, arm] = np.random.choice(matches, 2, replace=False)

            #calculate scale and angle using these matches
            scale, angle = self.calc_scale_angle(anchor, arm, kpq, kpt)

            #anchor coordinates 
            idx_q = anchor.queryIdx
            idx_t = anchor.trainIdx

            (yq_anch, xq_anch) = kpq[idx_q].pt
            (yt_anch, xt_anch) = kpt[idx_t].pt

            inlier_count = 0
            inlier_set = []

            # go through all matches, and check if inlier or outlier
            for mat in matches:

                idx_q = mat.queryIdx
                idx_t = mat.trainIdx

                (yq1, xq1) = kpq[idx_q].pt
                (yt1, xt1) = kpt[idx_t].pt

                #transform the match 
                dx, dy = self.transform(xq1, yq1, xq_anch, yq_anch, scale, angle)

                #calculate prediction
                xpred = xt_anch + dx
                ypred = yt_anch + dy

                #calculate error 
                error = np.linalg.norm([xpred -xt1, ypred - yt1])

                #check if error is less than threshold, if yes then inlier
                if error <  self.thres:
                    inlier_count += 1
                    inlier_set.append(mat)

            #keep largest set of inliers
            if inlier_count > max_count:
                best_count = inlier_count
                max_count = inlier_count
                best_angle = angle
                best_set = inlier_set
                best_anchor = anchor

        #calculate the average angle from the largest inlier set
        angles = []
        idx_q = best_anchor.queryIdx
        (yq_anch, xq_anch) = kpq[idx_q].pt
        for mat in best_set:
            idx_q = mat.queryIdx
            (yq1, xq1) = kpq[idx_q].pt
            if np.linalg.norm([xq1 -xq_anch, yq1 - yq_anch]) < 0.01:
                continue
            _, angle = self.calc_scale_angle(best_anchor, mat, kpq, kpt)
            angles.append(angle)

        avg_angle = np.average(angles)

        return best_count, avg_angle
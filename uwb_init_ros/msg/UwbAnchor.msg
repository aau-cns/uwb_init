int32               id          # anchor_id
int32               ref_id      #
geometry_msgs/Point position    #
float64             gamma       # constant bias    (pair-wise: anchor_id <-> ref_id)
float64             beta        # range-based bias (pair-wise: anchor_id <-> ref_id)
float64[15]         covariance  # position uncertainty

#ifndef PHY_OP_HPP
#define PHY_OP_HPP

#include <vector>
#include <complex>
#include <numeric>
#include <algorithm>

#include "vec_op.hpp"
#include "mat_op.hpp"

namespace phy_op{
    template<class T>
        std::vector<std::vector<T>> I(const std::vector<T>&v1,std::vector<T>&v2,std::vector<T>&v3,const T &m){
            std::vector<std::vector<T>>vlist={v1,v2,v3};
            int dim=vlist.size();
            if(dim==3){
                std::cout<<"dim is 3."<<std::endl;
                std::vector<std::vector<T>> ans({{0,0,0},{0,0,0},{0,0,0}});
                for(int i=0;i<dim;i++){
                    ans=ans+(mat_op::dot(mat_op::cross(vlist[i]),mat_op::cross(vlist[i])));
                    
                }
                for(int i=0;i<dim;i++){
                    ans=ans+(0.5)*(
                        mat_op::dot(
                            mat_op::cross(vlist[i%dim]),
                            mat_op::cross(vlist[(i+1)%dim])
                        )
                        +mat_op::dot(
                            mat_op::cross(vlist[(i+1)%dim]),
                            mat_op::cross(vlist[i%dim])
                        )
                    );
                }

                //std::cout<<"coefficient : "<<(((double)m/(-6*60)))<<std::endl;
                ans=((double)m/(-6*60))*ans;
                
                //std::cout<<"m : "<<m<<std::endl;
                //std::cout<<"W.I.P. Inertia : "<<std::endl;
                //for(int i=0;i<3;i++){
                //    for(int j=0;j<3;j++){
                //        std::cout<<i<<", "<<j<<" : "<<ans[i][j]<<" ";
                //    }
                //    std::cout<<std::endl;
                //}


                return ans;
            }
            //else if(dim==2){
            //    return m;
            //}
            else{
                return {{0,0,0},{0,0,0},{0,0,0}};
            }
        }

    
};

#endif
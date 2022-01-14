#ifndef MAT_OP_HPP
#define MAT_OP_HPP

#include <vector>
#include <complex>
#include <numeric>
#include <algorithm>

namespace mat_op{
    template<class T>//remember to generalize
        T det(const std::vector<std::vector<T>>&v1){
            T ans(0);//,escape(0);
            if(v1.size()==3){
                ans=v1[0][0]*v1[1][1]*v1[2][2]
                +v1[0][1]*v1[1][2]*v1[2][0]
                +v1[0][2]*v1[1][0]*v1[2][1]
                -v1[0][0]*v1[1][2]*v1[2][1]
                -v1[0][1]*v1[1][0]*v1[2][2]
                -v1[0][2]*v1[1][1]*v1[2][0];
            }else if(v1.size()==2){
                ans=v1[0][0]*v1[1][1]-v1[0][1]*v1[1][0];
            }
            return ans;
        }
    
    template<class T>
        std::vector<std::vector<T>> dot(const std::vector<std::vector<T>>&v1,const std::vector<std::vector<T>>&v2){
            std::vector<std::vector<T>> ans=v1;
            if(v1[0].size()==v2.size()){
                for(size_t i=0,row=ans.size();i<row;++i){
                    for(size_t j=0,col=ans[0].size();j<col;++j){
                        ans[i][j]=0;
                        for(size_t k=0;k<v1[0].size();++k){
                            ans[i][j]+=v1[i][k]*v2[k][j];
                        }
                    }
                }
                //return ans;
            }
            return ans;
        }
    
    template<class T>
        std::vector<std::vector<T>> cross(const std::vector<T>&v1){
            if(v1.size()==3){
                std::vector<std::vector<T>>// ans(3,3);
                ans={
                    {0,v1[2],-v1[1]},
                    {-v1[2],0,v1[0]},
                    {v1[1],-v1[0],0}
                };
                return ans;
            }             
            return {{0,0,0},{0,0,0},{0,0,0}};
        }

    template<class T>
        std::vector<std::vector<T>> inverse(const std::vector<std::vector<T>>&v1){
            if(mat_op::det(v1)!=0){
                if(v1.size()==3){
                    std::vector<std::vector<T>>ans=v1;
                    for(size_t i=0,row=ans.size();i<row;++i){
                        for(size_t j=0,col=ans[0].size();j<col;++j){
                            ans[i][j]=(
                                v1[(i+1)%3][(j+1)%3]*v1[(i+2)%3][(j+2)%3]
                                -v1[(i+1)%3][(j+2)%3]*v1[(i+2)%3][(j+1)%3]
                            )/mat_op::det(v1);
                        }
                    }
                    return ans;
                }
                else if(v1.size()==2){
                    T det=mat_op::det(v1);
                    std::vector<std::vector<T>>
                    ans={
                        {v1[1][1]/det,-v1[0][1]/det},
                        {-v1[1][0]/det,v1[0][0]/det}
                    };
                    return ans;
                }
            }
            return {{0,0,0},{0,0,0},{0,0,0}};
        }
};

template<class T>
    std::vector<std::vector<T>> operator+(const std::vector<std::vector<T>>&v1,const std::vector<std::vector<T>>&v2){
        std::vector<std::vector<T>>ans=v1;
        for(size_t i=0,size=ans.size();i<size;++i){
            for(size_t j=0;j<ans[0].size();++j){
                ans[i][j]+=v2[i][j];
            }
        }
        return ans;
    }

template<class T>
    std::vector<std::vector<T>> operator-(const std::vector<std::vector<T>>&v1,const std::vector<std::vector<T>>&v2){
        std::vector<std::vector<T>>ans=v1;
        for(size_t i=0,size=ans.size();i<size;++i){
            for(size_t j=0;j<ans[0].size();++j){
                ans[i][j]-=v2[i][j];
            }
        }
        return ans;
    }

template<class T>
    std::vector<std::vector<T>> operator*(const T a,const std::vector<std::vector<T>>&v1){
        std::vector<std::vector<T>>ans=v1;
        for(size_t i=0;i<ans.size();++i){
            for(size_t j=0;j<ans[0].size();++j){
                ans[i][j]*=a;
            }
        }
        return ans;
    }

template<class T>
    bool operator==(const std::vector<std::vector<T>>&v1,const std::vector<std::vector<T>>&v2){
        if(v1.size()==v2.size()&&v1[0].size()==v2[0].size()){
            bool flag=true;
            for(size_t i=0,size=v1.size();i<size;++i){
                for(size_t j=0;j<v1[0].size();++j){
                    flag=flag&&(v1[i][j]==v2[i][j]);
                }
            }
            return flag;
        }else{
            return false;
        }
    }





#endif
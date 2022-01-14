#ifndef VEC_OP_HPP
#define VEC_OP_HPP

#include <vector>
#include <complex>
#include <numeric>
#include <algorithm>

namespace vec_op{
    template<class T>
        T norm(const std::vector<T>&v1){
            T ans(0);
            for(size_t i=0,size=v1.size();i<size;++i){
                ans+=v1[i]*v1[i];
            }
            return sqrt(ans);
        }
    
    template<class T>
        T dot(const std::vector<T>&v1,const std::vector<T>&v2){
            T ans(0);
            for(size_t i=0,size=v1.size();i<size;++i){
                ans+=v1[i]*v2[i];
            }
            return ans;
        }

    template<class T>
        std::vector<T> cross(const std::vector<T>&v1,const std::vector<T>&v2){
            std::vector<T> ans({0,0,0});
            for(size_t i=0,size=v1.size();i<size;++i){
                ans[i]=v1[(i+1)%size]*v2[(i+2)%size]-v2[(i+1)%size]*v1[(i+2)%size];
            }
        return ans;
        }
    
    template<class T>
        T cross_2(const std::vector<T>&v1,const std::vector<T>&v2){
            //if(v1.size()==2&&v2.size()){
            return v1[0]*v2[1]-v2[0]*v1[1];
            
        }     
    template<class T>
        std::vector<T> mat(const std::vector<std::vector<T>>&M,const std::vector<T>&v1){
            std::vector<T> ans({0,0,0});
            for(size_t i=0,size=ans.size();i<size;++i){
                for(size_t j=0;j<size;++j){
                    ans[i]+=M[i][j]*v1[j];
                }
            }
            return ans;
        }
    
    template<class T>
        std::vector<T> rot_2(const T th,const std::vector<T> &v1){
            std::vector<T> ans=v1;
            ans[0]=v1[0]*cos(th)-v1[1]*sin(th);
            ans[1]=v1[0]*sin(th)+v1[1]*cos(th);
            return ans;
        }
};

template<class T>
    std::vector<T> operator+(const std::vector<T> &v1,const std::vector<T> &v2){
        std::vector<T> ans=v1;
        for(size_t i=0,size=ans.size();i<size;++i){
            ans[i]+=v2[i];
        }
        return ans;
    }

template<class T>
    std::vector<T> operator-(const std::vector<T> &v1,const std::vector<T> &v2){
        std::vector<T>ans=v1;
        for(size_t i=0,size=ans.size();i<size;++i){
            ans[i]-=v2[i];
        }
        return ans;
    }

template<class T>
    std::vector<T> operator*(const T a,const std::vector<T>&v1){
        std::vector<T>ans=v1;
        for(size_t i=0,size=ans.size();i<size;++i){
            ans[i]*=a;
        }
        return ans;
    }

template<class T>
    bool operator==(const std::vector<T>&v1,const std::vector<T>&v2){
        if(v1.size()==v2.size()){
            bool flag=true;
            for(size_t i=0,size=v1.size();i<size;++i){
                flag=flag&&(v1[i]==v2[i]);
            }
            return flag;
        }else{
            return false;
        }
    }
#endif
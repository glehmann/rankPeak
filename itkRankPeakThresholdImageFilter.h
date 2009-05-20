/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkRankPeakThresholdImageFilter.h,v $
  Language:  C++
  Date:      $Date: 2009-04-28 14:36:34 $
  Version:   $Revision: 1.9 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkRankPeakThresholdImageFilter_h
#define __itkRankPeakThresholdImageFilter_h

#include "itkImageToImageFilter.h"

namespace itk {

/** \class RankPeakThresholdImageFilter
 * \brief Produce a binary image by thresholding the input image with a value based on the maximum intensity of the peaks
 *
 * This filter automatically compute a threshold value based on the content of the input image, and use it
 * to produce the output image by thresholding the input one.
 *
 * The filter is able to compute a threshold even if the peaks are very thin and are mostly non represented
 * in the image. It uses some a priori knowledge on the content of the image. The most important is to give
 * a rough approximation of the number of peaks expected.
 *
 * The threshold is computed by taking N brightest maxima in the image, computing a percentile on those
 * value - by default, the median - and finally scaling this percentile.
 * 
 * \author Gaetan Lehmann. Biologie du Developpement et de la Reproduction, INRA de Jouy-en-Josas, France.
 *
 * \ingroup ImageEnhancement  MathematicalMorphologyImageFilters
 */
template<class TInputImage, class TOutputImage, class TAttribute=ITK_TYPENAME TInputImage::SpacingType::ValueType>
class ITK_EXPORT RankPeakThresholdImageFilter : 
    public ImageToImageFilter<TInputImage, TOutputImage>
{
public:
  /** Standard class typedefs. */
  typedef RankPeakThresholdImageFilter                  Self;
  typedef ImageToImageFilter<TInputImage, TOutputImage> Superclass;
  typedef SmartPointer<Self>                            Pointer;
  typedef SmartPointer<const Self>                      ConstPointer;

  /** Some convenient typedefs. */
  typedef TInputImage                              InputImageType;
  typedef typename InputImageType::Pointer         InputImagePointer;
  typedef typename InputImageType::ConstPointer    InputImageConstPointer;
  typedef typename InputImageType::RegionType      InputImageRegionType;
  typedef typename InputImageType::PixelType       InputImagePixelType;

  typedef TOutputImage                             OutputImageType;
  typedef typename OutputImageType::Pointer        OutputImagePointer;
  typedef typename OutputImageType::ConstPointer   OutputImageConstPointer;
  typedef typename OutputImageType::RegionType     OutputImageRegionType;
  typedef typename OutputImageType::PixelType      OutputImagePixelType;
  
  typedef TAttribute                               AttributeType;

  /** ImageDimension constants */
  itkStaticConstMacro(InputImageDimension, unsigned int,
                      TInputImage::ImageDimension);
  itkStaticConstMacro(OutputImageDimension, unsigned int,
                      TOutputImage::ImageDimension);
  itkStaticConstMacro(ImageDimension, unsigned int,
                      TOutputImage::ImageDimension);

  /** Standard New method. */
  itkNewMacro(Self);  

  /** Runtime information support. */
  itkTypeMacro(RankPeakThresholdImageFilter,
               ImageToImageFilter);

  /**
   * Set/Get whether the connected components are defined strictly by
   * face connectivity or by face+edge+vertex connectivity.  Default is
   * FullyConnectedOff.  For objects that are 1 pixel wide, use
   * FullyConnectedOn.
   */
  itkSetMacro(FullyConnected, bool);
  itkGetConstReferenceMacro(FullyConnected, bool);
  itkBooleanMacro(FullyConnected);

  /**
   * Set/Get the how the intensity value will be scaled to compute the final
   * threshold. Scale defaults to 0.5.
   */
  itkSetMacro(Scale, double);
  itkGetConstMacro(Scale, double);

  /**
   * Set/Get the number of peaks used to compute the threshold. NumberOfPeaks
   * defaults to 10.
   */
  itkSetMacro(NumberOfPeaks, int);
  itkGetConstMacro(NumberOfPeaks, int);

  /**
   * Set/Get the rank used to compute the threshold. Rank defaults to 0.5.
   */
  itkSetMacro(Rank, double);
  itkGetConstMacro(Rank, double);

  /**
   * Set/Get the foreground value of the output binary image. ForegroundValue defaults to the maximum value of
   * the output type.
   */
  itkSetMacro(ForegroundValue, OutputImagePixelType);
  itkGetConstMacro(ForegroundValue, OutputImagePixelType);

  /**
   * Set/Get the background value of the output binary image. BackgroundValue defaults to zero.
   */
  itkSetMacro(BackgroundValue, OutputImagePixelType);
  itkGetConstMacro(BackgroundValue, OutputImagePixelType);

  /**
   * Get the threshold computed in this filter. Threshold is valid only after the update of the filter.
   */
  itkGetConstMacro(Threshold, InputImagePixelType);

protected:
  RankPeakThresholdImageFilter();
  ~RankPeakThresholdImageFilter() {};
  void PrintSelf(std::ostream& os, Indent indent) const;

  /** RankPeakThresholdImageFilter needs the entire input be
   * available. Thus, it needs to provide an implementation of
   * GenerateInputRequestedRegion(). */
  void GenerateInputRequestedRegion();

  void GenerateData();
  

private:
  RankPeakThresholdImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  bool                 m_FullyConnected;
  double               m_Rank;
  double               m_Scale;
  int                  m_NumberOfPeaks;
  OutputImagePixelType m_BackgroundValue;
  OutputImagePixelType m_ForegroundValue;
  InputImagePixelType  m_Threshold;
  
}; // end of class

} // end namespace itk
  
#ifndef ITK_MANUAL_INSTANTIATION
#include "itkRankPeakThresholdImageFilter.txx"
#endif

#endif

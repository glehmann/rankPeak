/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkRankPeakThresholdImageFilter.txx,v $
  Language:  C++
  Date:      $Date: 2009-01-08 16:03:55 $
  Version:   $Revision: 1.4 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkRankPeakThresholdImageFilter_txx
#define __itkRankPeakThresholdImageFilter_txx

#include "itkRankPeakThresholdImageFilter.h"
#include "itkRegionalMaximaImageFilter.h"
#include "itkBinaryImageToStatisticsLabelMapFilter.h"
#include "itkStatisticsRelabelLabelMapFilter.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkProgressAccumulator.h"


namespace itk {

template <class TInputImage, class TOutputImage, class TAttribute>
RankPeakThresholdImageFilter<TInputImage, TOutputImage, TAttribute>
::RankPeakThresholdImageFilter()
{
  m_FullyConnected = false;
  m_Rank = 0.5;
  m_Scale = 0.5;
  m_NumberOfPeaks = 10;
  m_BackgroundValue = NumericTraits<OutputImagePixelType>::Zero;
  m_ForegroundValue = NumericTraits<OutputImagePixelType>::max();
  m_Threshold = NumericTraits<InputImagePixelType>::Zero;
}

template <class TInputImage, class TOutputImage, class TAttribute>
void 
RankPeakThresholdImageFilter<TInputImage, TOutputImage, TAttribute>
::GenerateInputRequestedRegion()
{
  // call the superclass' implementation of this method
  Superclass::GenerateInputRequestedRegion();
  
  // We need all the input.
  InputImagePointer input = const_cast<InputImageType *>(this->GetInput());
  
  input->SetRequestedRegion( input->GetLargestPossibleRegion() );
}


template <class TInputImage, class TOutputImage, class TAttribute>
void 
RankPeakThresholdImageFilter<TInputImage, TOutputImage, TAttribute>
::GenerateData()
{
  // Create a process accumulator for tracking the progress of this minipipeline
  ProgressAccumulator::Pointer progress = ProgressAccumulator::New();
  progress->SetMiniPipelineFilter(this);

  // Allocate the output
  this->AllocateOutputs();
  
  // Internal image type
  typedef Image< unsigned char, ImageDimension > InternalImageType;
  
  // search the maxima in the image
  typedef RegionalMaximaImageFilter<InputImageType, InternalImageType> MaximaType;
  typename MaximaType::Pointer rmax = MaximaType::New();
  rmax->SetInput( this->GetInput() );
  rmax->SetFullyConnected( m_FullyConnected );
  rmax->SetNumberOfThreads( this->GetNumberOfThreads() );
  progress->RegisterInternalFilter(rmax,.7f);
  
  // get the regional maxima as a label map
  typedef BinaryImageToStatisticsLabelMapFilter<InternalImageType, TInputImage> B2LMType;
  typedef typename B2LMType::OutputImageType LabelMapType;
  typename B2LMType::Pointer b2lm = B2LMType::New();
  b2lm->SetInput( rmax->GetOutput() );
  b2lm->SetFeatureImage( this->GetInput() );
  b2lm->SetFullyConnected( m_FullyConnected );
  b2lm->SetNumberOfThreads( this->GetNumberOfThreads() );
  progress->RegisterInternalFilter(b2lm,.1f);
  
  // reorder the label object according to there intensity to make it easy to get the rankth
  // one
  typedef StatisticsRelabelLabelMapFilter< LabelMapType > RelabelType;
  typename RelabelType::Pointer relabel = RelabelType::New();
  relabel->SetInput( b2lm->GetOutput() );
  relabel->SetAttribute( RelabelType::LabelObjectType::MAXIMUM );
  relabel->SetNumberOfThreads( this->GetNumberOfThreads() );
  progress->RegisterInternalFilter(relabel,.1f);

  // get the value we're interested in from that list
  relabel->Update();
  
  InputImagePixelType rankVal = relabel->GetOutput()->GetNthLabelObject( m_NumberOfPeaks * m_Rank )->GetMaximum();
  m_Threshold = static_cast< InputImagePixelType >( rankVal * m_Scale );

  // threshold the input image with the computed threshold
  typedef BinaryThresholdImageFilter< InputImageType, OutputImageType > ThresholdType;
  typename ThresholdType::Pointer th = ThresholdType::New();
  th->SetInput( this->GetInput() );
  th->SetLowerThreshold( m_Threshold );
  th->SetInsideValue( m_ForegroundValue );
  th->SetOutsideValue( m_BackgroundValue );
  th->SetNumberOfThreads( this->GetNumberOfThreads() );
  progress->RegisterInternalFilter(th,.1f);
  
  // graft our output to the subtract filter to force the proper regions
  // to be generated
  th->GraftOutput( this->GetOutput() );

  // run the algorithm
  th->Update();

  // graft the output of the subtract filter back onto this filter's
  // output. this is needed to get the appropriate regions passed
  // back.
  this->GraftOutput( th->GetOutput() );

}

template<class TInputImage, class TOutputImage, class TAttribute>
void
RankPeakThresholdImageFilter<TInputImage, TOutputImage, TAttribute>
::PrintSelf(std::ostream &os, Indent indent) const
{
  Superclass::PrintSelf(os, indent);

  os << indent << "FullyConnected: " << m_FullyConnected << std::endl;
  os << indent << "Rank: " << m_Rank << std::endl;
  os << indent << "Scale: " << m_Scale << std::endl;
  os << indent << "NumberOfPeaks: " << m_NumberOfPeaks << std::endl;
  os << indent << "BackgroundValue: " << static_cast< typename NumericTraits< OutputImagePixelType >::PrintType>( m_BackgroundValue ) << std::endl;
  os << indent << "ForegroundValue: " << static_cast< typename NumericTraits< OutputImagePixelType >::PrintType>( m_ForegroundValue ) << std::endl;
  os << indent << "Threshold: " << static_cast< typename NumericTraits< InputImagePixelType >::PrintType>( m_Threshold ) << std::endl;
}

}// end namespace itk
#endif
